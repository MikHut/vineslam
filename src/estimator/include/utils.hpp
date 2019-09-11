#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#define pi 3.14159265359

struct Parameters {
  double h_fov;      /* Camera horizontal field of view */
  double v_fov;      /* Camera vertical field of view */
  double cam_height; /* Camera height (meters) */
  int    width;      /* Image width. */
  int    height;     /* Image height */
  int resolution; /* Grid resolution =  (resolution x 100, resolutions x 100) */
  int match_box;  /* Search box diagonal size */

  Parameters()
  {
    h_fov      = pi / 2;
    v_fov      = pi / 2;
    cam_height = 1.0;
    width      = 1280;
    height     = 960;
    resolution = 10;
    match_box  = 10;
  }
};

template <typename T>
struct Point {
  T x;
  T y;

  Point() {}

  Point(const T x, const T y)
  {
    (*this).x = x;
    (*this).y = y;
  }

  Point(const Point<T>& pt)
  {
    (*this).x = pt.x;
    (*this).y = pt.y;
  }

  double euc_dist(const Point<T>& pt)
  {
    return sqrt(((*this).x - pt.x) * ((*this).x - pt.x) +
		((*this).y - pt.y) * ((*this).y - pt.y));
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& o, const Point<T>& pt)
{
  o << "[x,y] = [" << pt.x << "," << pt.y << "]" << std::endl;
  return o;
}

template <typename T>
std::ostream& operator<<(std::ostream& o, const std::vector<Point<T>>& pt)
{
  for (size_t i = 0; i < pt.size(); i++)
    o << "[x,y] = [" << pt[i].x << "," << pt[i].y << "]" << std::endl;
  return o;
}

template <typename T>
struct Match {
  Point<T> p;
  Point<T> c;
  double   p_ang;
  double   c_ang;

  Match(const Point<T>& p, const Point<T>& c, const Parameters& params)
  {
    (*this).p     = Point<T>(p);
    (*this).c     = Point<T>(c);
    (*this).p_ang = atan2(params.height - p.y, p.x - params.width / 2);
    (*this).c_ang = atan2(params.height - c.y, c.x - params.width / 2);
  }
};
