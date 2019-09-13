#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#define PI 3.14159265359	 /* (radians) */
#define RAD PI / 180		 /* one radian */
#define TRUNK_SCOPE 7.5		 /* maximum distance of trunk detection (meters) */
#define MAX_DXY 0.2		 /* maximum displacement per iteration (meters) */
#define MAX_DTHETA 10 * PI / 180 /* maximum rotation per iteration (radians) */

struct Parameters {
  double h_fov;      /* Camera horizontal field of view */
  double v_fov;      /* Camera vertical field of view */
  double cam_height; /* Camera height (meters) */
  int    width;      /* Image width. */
  int    height;     /* Image height */
  int    resolution; /* Grid resolution =  (resolution x 100, resolutions x 100) */
  int    match_box;  /* Search box diagonal size */

  Parameters()
  {
    h_fov      = PI / 4;
    v_fov      = PI / 4;
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
  Point<T>		p;
  Point<T>		c;
  std::vector<Point<T>> p_line;
  std::vector<Point<T>> c_line;
  double		p_ang;
  double		c_ang;

  Match(const Point<T>& p, const Point<T>& c, const std::vector<Point<T>> p_line,
	const std::vector<Point<T>> c_line, const Parameters& params)
  {
    (*this).p      = Point<T>(p);
    (*this).c      = Point<T>(c);
    (*this).p_ang  = ((params.h_fov / 2) / params.width) * (p.x - params.width / 2);
    (*this).c_ang  = ((params.h_fov / 2) / params.width) * (c.x - params.width / 2);
    (*this).p_line = p_line;
    (*this).c_line = c_line;
  }
};

template <typename T>
struct Particle {
  int      id;
  Point<T> pos;
  double   theta;
  double   weight;

  Particle(const int& id, const Point<T>& pos, const double& theta, const double& weight)
  {
    (*this).id     = id;
    (*this).pos    = pos;
    (*this).theta  = theta;
    (*this).weight = weight;
  }
};
