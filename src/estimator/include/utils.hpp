#pragma once

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

const float INF = 1.0e6; /* hypothetic infinit */
const float TRUNK_SCOPE =
    7.5 * 100; /* maximum distance of trunk detection (centimeters) */
const float PI     = 3.14159265359;    /* (radians) */
const float DEGREE = 1.0 * PI / 180.0; /* one radian */
const float STD_XY = 0.05; /* standard deviation of delta[x,y] (centimeters) */
const float STD_THETA =
    0.001 * PI / 180.0; /* standard deviation of delta_theta (radians) */
const float MEAN_X =
    0.1; /* initial mean for delta[x] displacement (centimeters) */
const float MEAN_Y =
    0.0; /* initial mean for delta[y] displacement (centimeters) */
const float MEAN_THETA = 0.0; /*initial mean for delta_theta (radians) */

struct Parameters
{
	double h_fov;      /* Camera horizontal field of view */
	double v_fov;      /* Camera vertical field of view */
	double cam_height; /* Camera height (centimenters) */
	int    width;      /* Image width. */
	int    height;     /* Image height */
	int    resolution; /* Grid resolution =  (resolution, resolution) cm */
	int    match_box;  /* Search box diagonal size */

	Parameters()
	{
		h_fov      = PI / 4;
		v_fov      = PI / 4;
		cam_height = 100.0;
		width      = 1280;
		height     = 960;
		resolution = 1000;
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
struct Pose
{
	Point<T> pos;
	double   theta;

	Pose() {}

	Pose(const Point<T>& pos, const double& theta)
	{
		(*this).pos   = pos;
		(*this).theta = theta;
	}
};

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
struct Landmark
{
	int                   id;
	Point<T>              world_pos;
	Point<T>              image_pos;
	std::vector<Point<T>> unc;

	Landmark() {}

	Landmark(const int& id, const Point<T>& world_pos, const Point<T>& image_pos,
	         const std::vector<Point<double>>& unc)
	{
		(*this).id        = id;
		(*this).world_pos = world_pos;
		(*this).image_pos = image_pos;
		(*this).unc       = unc;
	}

	void updateUnc(const std::vector<Point<double>> &unc)
	{
		(*this).unc.insert((*this).unc.end(), unc.begin(), unc.end());
	}
};

template <typename T>
struct Match
{
	Point<T> p_pos;
	Point<T> c_pos;
	Line<T>  p_line;
	Line<T>  c_line;

	Match() {}

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

	Particle() {}

	Particle(const int& id, const Point<T>& delta_p, const double& delta_th,
	         const double& weight)
	{
		(*this).id       = id;
		(*this).delta_p  = delta_p;
		(*this).delta_th = delta_th;
		(*this).weight   = weight;
	}
};

/* ----- operators ----- */

template <typename T1, typename T2>
Point<T1> operator+(const Point<T1>& p1, const Point<T2>& p2)
{
	return Point<T1>(p1.x + p2.x, p1.y + p2.y);
}

template <typename T1, typename T2>
Point<T1> operator-(const Point<T1>& p1, const Point<T2>& p2)
{
	return Point<T1>(p1.x - p2.x, p1.y - p2.y);
}

template <typename T1, typename T2>
Point<T1> operator/(const Point<T1>& p1, const Point<T2>& p2)
{
	return Point<T1>(p1.x / p2.x, p1.y / p2.y);
}

template <typename T1, typename T2>
Point<T1> operator/(const Point<T1>& p1, const T2& c)
{
	return Point<T1>(p1.x / c, p1.y / c);
}

template <typename T1, typename T2>
Point<T1> operator*(const Point<T1>& p1, const Point<T2>& p2)
{
	return Point<T1>(p1.x * p2.x, p1.y * p2.y);
}

template <typename T1, typename T2>
Point<T1> operator*(const Point<T1>& p1, const T2& c)
{
	return Point<T1>(p1.x * c, p1.y * c);
}

template <typename T1, typename T2>
bool operator==(const Point<T1>& p1, const Point<T2>& p2)
{
	return (p1.x == p2.x) && (p1.y == p2.y);
}

template <typename T>
std::ostream& operator<<(std::ostream& o, const Point<T>& pt)
{
	o << "[x,y] = [" << pt.x << "," << pt.y << "]" << std::endl;
	return o;
}

template <typename T>
std::ostream& operator<<(std::ostream& o, const Line<T>& l)
{
	o << l.a << " * x + " << l.b << " * y = " << l.c << std::endl;
	return o;
}

template <typename T>
std::ostream& operator<<(std::ostream& o, const Particle<T>& p)
{
	o << "(" << p.id << ") - " << p.pos << "theta = " << p.theta
	  << "\nweight = " << p.weight << std::endl;
	return o;
}

template <typename T>
std::ostream& operator<<(std::ostream& o, const Pose<T>& p)
{
	o << "[x,y,theta] = [" << p.pos.x << "," << p.pos.y << "," << p.theta << "]"
	  << std::endl;
	return o;
}

template <typename T>
std::ostream& operator<<(std::ostream& o, const Landmark<T>& l)
{
	o << "[x,y] = [" << l.pos.x << "," << l.pos.y << "]" << std::endl;
	return o;
}
