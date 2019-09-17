#pragma once

#include <cmath>
#include <iostream>
#include <vector>

const float INF = 1.0e6; /* hypothetic infinit */
const float TRUNK_SCOPE =
    7.5; /* maximum distance of trunk detection (meters) */
const float PI      = 3.14159265359; /* (radians) */
const float RAD     = PI / 180.0;    /* one radian */
const float MAX_DXY = 0.5; /* maximum displacement per iteration (meters) */
const float MAX_DTHETA =
    10.0 * PI / 180.0; /* maximum rotation per iteration (radians) */

struct Parameters
{
	double h_fov;      /* Camera horizontal field of view */
	double v_fov;      /* Camera vertical field of view */
	double cam_height; /* Camera height (meters) */
	int    width;      /* Image width. */
	int    height;     /* Image height */
	int resolution; /* Grid resolution =  (resolution x 100, resolutions x 100) */
	int match_box;  /* Search box diagonal size */

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
struct Point
{
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
struct Line
{
	/* ax + by = c */
	T a;
	T b;
	T c;

	Point<T> p1;
	Point<T> p2;

	Line() {}

	Line(const Point<T>& p1, const Point<T>& p2)
	{
		a = p2.y - p1.y;
		b = p1.x - p2.x;
		c = a * p1.x + b * p1.y;

		(*this).p1 = p1;
		(*this).p2 = p2;
	}

	Point<T> intercept(const Line<T>& l2)
	{
		Line<T> l1  = *this;
		double  det = l1.a * l2.b - l2.a * l1.b;

		if (det == 0)
			return Point<T>(INF, INF);
		else {
			double x = (l2.b * l1.c - l1.b * l2.c) / det;
			double y = (l1.a * l2.c - l2.a * l1.c) / det;

			return Point<T>(x, y);
		}
	}
};

template <typename T>
std::ostream& operator<<(std::ostream& o, const Line<T>& l)
{
	o << l.a << " * x + " << l.b << " * y = " << l.c << std::endl;
	return o;
}

template <typename T>
struct Match
{
	Point<T> p_pos;
	Point<T> c_pos;
	Line<T>  p_line;
	Line<T>  c_line;

	Match(const Point<T>& p, const Point<T>& c, const Line<T>& p_line,
	      const Line<T>& c_line)
	{
		(*this).p_pos  = Point<T>(p);
		(*this).c_pos  = Point<T>(c);
		(*this).p_line = p_line;
		(*this).c_line = c_line;
	}
};

template <typename T>
struct Particle
{
	int      id;
	Point<T> pos;
	double   theta;
	double   weight;

	Particle(const int& id, const Point<T>& pos, const double& theta,
	         const double& weight)
	{
		(*this).id     = id;
		(*this).pos    = pos;
		(*this).theta  = theta;
		(*this).weight = weight;
	}
};

template <typename T>
std::ostream& operator<<(std::ostream& o, const Particle<T>& p)
{
	o << "(" << p.id << ") - " << p.pos << "theta = " << p.theta
	  << "\nweight = " << p.weight << std::endl;
	return o;
}
