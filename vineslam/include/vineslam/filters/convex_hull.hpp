#pragma once

#include <iostream>
#include <vector>
#include <stack>
#include <cmath>

#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/math/Point.hpp>

namespace vineslam
{
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
static int orientation(const Point& p, const Point& q, const Point& r)
{
  float val = (q.y_ - p.y_) * (r.x_ - q.x_) - (q.x_ - p.x_) * (r.y_ - q.y_);

  if (std::fabs(val) < 1e-20)
    return 0;                // colinear
  return (val > 0) ? 1 : 2;  // clock or counterclock wise
}

// Compare function
static Point min_pt{};
static int compare(const void* vp1, const void* vp2)
{
  auto* p1 = (Point*)vp1;
  auto* p2 = (Point*)vp2;

  int o = orientation(min_pt, *p1, *p2);

  if (o == 0)
  {
    return (min_pt.distanceXY(*p2) >= min_pt.distanceXY(*p1)) ? -1 : 1;
  }

  return (o == 2) ? -1 : 1;
}

// A utility function to find next to top in a stack
static Point nextToTop(std::stack<Point>& S)
{
  if (S.size() < 2)
  {
    return Point(0, 0, 0);
  }

  Point p = S.top();
  S.pop();
  Point res = S.top();
  S.push(p);
  return res;
}

static bool convexHull(const Plane& plane, SemiPlane& semi_plane)
{
  if (plane.points_.empty())
  {
    return false;
  }

  // ------------------------------------------------------------------
  // ----- Get local plane reference frame transformation matrix
  // ------------------------------------------------------------------
  Tf plane_ref = plane.getLocalRefFrame();

  // ------------------------------------------------------------------
  // ----- Transform plane points to the local reference frame
  // ------------------------------------------------------------------
  int size = plane.points_.size();
  Point l_pts[size];
  for (int k = 0; k < size; k++)
  {
    Point l_pt = plane.points_[k] * plane_ref.inverse();
    l_pts[k] = l_pt;
  }

  // ------------------------------------------------------------------
  // ----- Apply the Convex Hull algorithm to get the semi-plane extremas
  // ----- from: https://www.geeksforgeeks.org/convex-hull-set-2-graham-scan/
  // ------------------------------------------------------------------

  // (A) - Find the bottom-most point
  float ymin = l_pts[0].y_;
  int min = 0, i = 1;
  for (const auto& pt : l_pts)
  {
    if (pt.y_ < ymin || (ymin == pt.y_ && pt.x_ < l_pts[min].x_))
    {
      ymin = pt.y_;
      min = i;
    }

    i++;
  }

  // (B) - Place the bottom-most point at the first position
  Point tmp = l_pts[0];
  l_pts[0] = l_pts[min];
  l_pts[min] = tmp;
  min_pt = l_pts[0];

  // (C) - Sort n-1 points with respect to the first point. A point p1 comes before p2 in sorted output if p2
  //       has larger polar angle (in counterclockwise direction) than p1
  qsort(&l_pts[1], size - 1, sizeof(Point), compare);

  // If two or more points make same angle with p0, remove all but the one that is farthest from p0
  // Remember that, in above sorting, our criteria was to keep the farthest point at the end when more than
  // one points have same angle.
  int m = 1;  // Initialize size of modified array
  for (int k = 1; k < size; k++)
  {
    // Keep removing i while angle of k and k+1 is same
    // with respect to p0
    while (k < size - 1 && orientation(min_pt, l_pts[k], l_pts[k + 1]) == 0)
      k++;

    l_pts[m] = l_pts[k];
    m++;  // Update size of modified array
  }

  // If modified array of points has less than 3 points, convex hull is not possible
  if (m < 3)
    return false;

  // (D) - Find the extrema points

  // Create an empty stack and push first three points to it.
  std::stack<Point> S;
  S.push(l_pts[0]);
  S.push(l_pts[1]);
  S.push(l_pts[2]);

  // Process remaining n-3 points
  for (int k = 3; k < m; k++)
  {
    // Keep removing top while the angle formed by
    // points next-to-top, top, and l_pts[k] makes
    // a non-left turn
    if (!S.empty())
    {
      while (orientation(nextToTop(S), S.top(), l_pts[k]) != 2)
      {
        if (!S.empty())
        {
          S.pop();

          if (S.empty())
            break;
        }
      }
    }
    S.push(l_pts[k]);
  }

  // Convert stack to vector and save the output semi-plane
  std::vector<Point> extremas;
  while (!S.empty())
  {
    Point p = S.top() * plane_ref;
    if (std::isfinite(p.x_) && std::isfinite(p.y_) && std::isfinite(p.z_))
    {
      extremas.push_back(p);
    }
    S.pop();
  }
  semi_plane = SemiPlane(plane, extremas);

  return true;
}

}  // namespace vineslam