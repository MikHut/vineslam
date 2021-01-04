#pragma once

#include <iostream>
#include <vector>
#include <stack>
#include <cmath>

#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/math/Point.hpp>

namespace vineslam
{
static bool convexHull(const Plane& plane, SemiPlane& semi_plane)
{
  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  auto orientation = [](const Point& p, const Point& q, const Point& r) {
    float val = (q.y_ - p.y_) * (r.x_ - q.x_) - (q.x_ - p.x_) * (r.y_ - q.y_);

    if (std::fabs(val) < 1e-52)
      return 0;                // colinear
    return (val > 0) ? 1 : 2;  // clock or counterclock wise
  };

  if (plane.points_.empty())
  {
    return false;
  }

  // ------------------------------------------------------------------
  // ----- Get local plane reference frame transformation matrix
  // ------------------------------------------------------------------
  Tf plane_ref = plane.local_ref_;
  Tf plane_ref_inv = plane_ref.inverse();

  // ------------------------------------------------------------------
  // ----- Transform plane points to the local reference frame
  // ------------------------------------------------------------------
  int n = plane.points_.size();
  std::vector<Point> l_pts(n);
  for (int k = 0; k < n; k++)
  {
    Point l_pt = plane.points_[k] * plane_ref_inv;
    l_pts[k] = l_pt;
  }

  // ------------------------------------------------------------------
  // ----- Apply the Convex Hull algorithm to get the semi-plane extremas
  // ----- from: https://www.geeksforgeeks.org/convex-hull-set-1-jarviss-algorithm-or-wrapping/
  // ------------------------------------------------------------------
  std::vector<Point> hull;

  // Find the leftmost point
  int l = 0;
  for (int i = 1; i < n; i++)
  {
    if (l_pts[i].x_ < l_pts[l].x_)
    {
      l = i;
    }
  }

  // Start from leftmost point, keep moving counterclockwise
  // until reach the start point again.  This loop runs O(h)
  // times where h is number of points in result or output.
  int p = l, q;
  do
  {
    // Add current point to result
    hull.push_back(l_pts[p]);

    // Search for a point 'q' such that orientation(p, x,
    // q) is counterclockwise for all points 'x'. The idea
    // is to keep track of last visited most counterclock-
    // wise point in q. If any point 'i' is more counterclock-
    // wise than q, then update q.
    q = (p + 1) % n;
    for (int i = 0; i < n; i++)
    {
      // If i is more counterclockwise than current q, then
      // update q
      if (orientation(l_pts[p], l_pts[i], l_pts[q]) == 2)
        q = i;
    }

    // Now q is the most counterclockwise with respect to p
    // Set p as q for next iteration, so that q is added to
    // result 'hull'
    p = q;

  } while (p != l);  // While we don't come to first point

  // Convert stack to vector and save the output semi-plane
  std::vector<Point> extremas;
  for (auto& pt : hull)
  {
    pt = pt * plane_ref;
    if (std::isfinite(pt.x_) && std::isfinite(pt.y_) && std::isfinite(pt.z_))
    {
      extremas.push_back(pt);
    }
  }
  semi_plane = SemiPlane(plane, extremas);

  return true;
}

// --------------------------------------------------------------------------------------------------------------------
// ----- Methods for convex polygon intersection
// ----- from: https://www.swtestacademy.com/intersection-convex-polygons-algorithm/
// --------------------------------------------------------------------------------------------------------------------

static void orderCounterClockWise(std::vector<Point>& pts)
{
  // Find the bottommost point
  int n = pts.size();
  float ymin = pts[0].y_;
  int min = 0;
  for (int i = 1; i < n; i++)
  {
    float y = pts[i].y_;

    // Pick the bottom-most or chose the left
    // most point in case of tie
    if ((y < ymin) || (ymin == y && pts[i].x_ < pts[min].x_))
      ymin = pts[i].y_, min = i;
  }

  // Place the bottom-most point at first position
  Point tmp = pts[0];
  pts[0] = pts[min];
  pts[min] = tmp;
  Point p0 = pts[0];

  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  auto orientation = [](const Point& p, const Point& q, const Point& r) {
    float val = (q.y_ - p.y_) * (r.x_ - q.x_) - (q.x_ - p.x_) * (r.y_ - q.y_);

    if (std::fabs(val) < 1e-52)
      return 0;                // colinear
    return (val > 0) ? 1 : 2;  // clock or counterclock wise
  };

  // A function used by the sort routine
  auto compare = [p0, orientation](const Point& p1, const Point& p2) {
    // Find orientation
    int o = orientation(p0, p1, p2);
    if (o == 0)
      return p0.sqDistanceXY(p2) < p0.sqDistanceXY(p1);

    return o != 2;
  };

  // Sort counterclockwise
  std::sort(pts.begin(), pts.end(), compare);
}

static bool intersectPolygonLines(const Point& l1p1, const Point& l1p2, const Point& l2p1, const Point& l2p2,
                                  Point& isct)
{
  float A1 = l1p2.y_ - l1p1.y_;
  float B1 = l1p1.x_ - l1p2.x_;
  float C1 = A1 * l1p1.x_ + B1 * l1p1.y_;

  float A2 = l2p2.y_ - l2p1.y_;
  float B2 = l2p1.x_ - l2p2.x_;
  float C2 = A2 * l2p1.x_ + B2 * l2p1.y_;

  // lines are parallel
  float det = A1 * B2 - A2 * B1;
  if (isEqual(det, 0))
  {
    return false;
  }
  else
  {
    float x = (B2 * C1 - B1 * C2) / det;
    float y = (A1 * C2 - A2 * C1) / det;
    bool online1 = ((std::min(l1p1.x_, l1p2.x_) < x || isEqual(std::min(l1p1.x_, l1p2.x_), x)) &&
                    (std::max(l1p1.x_, l1p2.x_) > x || isEqual(std::max(l1p1.x_, l1p2.x_), x)) &&
                    (std::min(l1p1.y_, l1p2.y_) < y || isEqual(std::min(l1p1.y_, l1p2.y_), y)) &&
                    (std::max(l1p1.y_, l1p2.y_) > y || isEqual(std::max(l1p1.y_, l1p2.y_), y)));
    bool online2 = ((std::min(l2p1.x_, l2p2.x_) < x || isEqual(std::min(l2p1.x_, l2p2.x_), x)) &&
                    (std::max(l2p1.x_, l2p2.x_) > x || isEqual(std::max(l2p1.x_, l2p2.x_), x)) &&
                    (std::min(l2p1.y_, l2p2.y_) < y || isEqual(std::min(l2p1.y_, l2p2.y_), y)) &&
                    (std::max(l2p1.y_, l2p2.y_) > y || isEqual(std::max(l2p1.y_, l2p2.y_), y)));

    if (online1 && online2)
    {
      isct = Point(x, y);
      return true;
    }
  }

  return false;
}

static bool isPointInsidePolygon(const Point& test, const SemiPlane& poly)
{
  size_t i;
  size_t j;
  bool result = false;
  for (i = 0, j = poly.extremas_.size() - 1; i < poly.extremas_.size(); j = i++)
  {
    if ((poly.extremas_[i].y_ > test.y_) != (poly.extremas_[j].y_ > test.y_) &&
        (test.x_ < (poly.extremas_[j].x_ - poly.extremas_[i].x_) * (test.y_ - poly.extremas_[i].y_) /
                           (poly.extremas_[j].y_ - poly.extremas_[i].y_) +
                       poly.extremas_[i].x_))
    {
      result = !result;
    }
  }
  return result;
}

static void getIntersectionPoints(const Point& l1p1, const Point& l1p2, const SemiPlane& poly, std::vector<Point>& isct)
{
  std::vector<Point> i_pts;
  for (size_t i = 0; i < poly.extremas_.size(); i++)
  {
    size_t next = (i + 1 == poly.extremas_.size()) ? 0 : i + 1;

    Point ip;
    if (intersectPolygonLines(l1p1, l1p2, poly.extremas_[i], poly.extremas_[next], ip))
    {
      isct.push_back(ip);
    }
  }
}

static bool polygonIntersection(const SemiPlane& S1, const SemiPlane& S2, std::vector<Point>& isct)
{
  //  // Add  the corners of poly1 which are inside poly2
  for (const auto& extrema : S1.extremas_)
  {
    if (isPointInsidePolygon(extrema, S2))
    {
      isct.push_back(extrema);
    }
  }

  // Add the corners of poly2 which are inside poly1
  for (const auto& extrema : S2.extremas_)
  {
    if (isPointInsidePolygon(extrema, S1))
    {
      isct.push_back(extrema);
    }
  }

  // Add  the intersection points
  for (int i = 0, next = 1; i < S1.extremas_.size(); i++, next = (i + 1 == S1.extremas_.size()) ? 0 : i + 1)
  {
    std::vector<Point> l_isct;
    getIntersectionPoints(S1.extremas_[i], S1.extremas_[next], S2, l_isct);
    if (!l_isct.empty())
    {
      isct.insert(isct.end(), l_isct.begin(), l_isct.end());
    }
  }

  if (!isct.empty())
  {
    orderCounterClockWise(isct);
  }

  return true;
}

}  // namespace vineslam