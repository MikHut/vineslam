#pragma once

#include <cmath>
#include <iostream>

struct parameters {
  int width;      /* Image width. */
  int height;     /* Image height */
  int resolution; /* Grid resolution =  (resolution x 100, resolutions x 100) */
  int match_box;  /* Search box diagonal size */

  parameters()
  {
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
struct Match {
  Point<T> p;
  Point<T> c;

  Match(const Point<T>& p, const Point<T>& c)
  {
    (*this).p = Point<T>(p);
    (*this).c = Point<T>(c);
  }
};
