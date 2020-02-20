#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching.hpp>
#include <random>
#include <vector>

const float INF = 1.0e6;         /* hypothetic infinit */
const float PI  = 3.14159265359; /* (radians) */

struct Parameters
{
	double h_fov;     /* Camera horizontal field of view (radians) */
	double v_fov;     /* Camera vertical field of view (radians) */
	int    width;     /* Image width */
	int    height;    /* Image height */
	double min_score; /* Minimum trunk detection probability */
	int    match_box; /* Local search matching box dimension */

	std::string image_right; /* right image ROS topic */
	std::string image_left;  /* right image ROS topic */
	std::string image_depth; /* depth image ROS topic */
	std::string odom_topic;  /* odometry ROS topic */
	std::string model;       /* tflite model path */
	std::string labels;      /* detection labels path */

	Parameters()
	{
		h_fov       = PI / 2;
		v_fov       = PI / 2;
		width       = 1280;
		height      = 960;
		min_score   = 0.5;
		match_box   = 20;
		model       = "";
		labels      = "";
		image_right = "";
		image_left  = "";
		image_depth = "";
		odom_topic  = "";
	}
};

/* structures */

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

	Eigen::VectorXd eig()
	{
		Eigen::VectorXd vec(2, 1);
		vec << x, y;
		return vec;
	}
};

template <typename T>
struct Pose
{
	Point<T> pos;
	double   theta;

	Pose() {}

	Pose(const T& x, const T& y, const double& theta)
	{
		(*this).pos.x = x;
		(*this).pos.y = y;
		(*this).theta = theta;
	}

	Pose(const Point<T>& pos, const double& theta)
	{
		(*this).pos   = pos;
		(*this).theta = theta;
	}

	Eigen::VectorXd eig()
	{
		Eigen::VectorXd vec(3, 1);
		vec << pos.x, pos.y, theta;
		return vec;
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

	Line(const std::vector<Point<T>>& pts)
	{
		if (pts.size() < 2)
			return;

		p1 = pts[0];
		p2 = pts[pts.size() - 1];

		T X  = 0;
		T Y  = 0;
		T XY = 0;
		T X2 = 0;
		T Y2 = 0;

		for (size_t i = 0; i < pts.size(); i++) {
			X += pts[i].x;
			Y += pts[i].y;
			XY += pts[i].x * pts[i].y;
			X2 += pts[i].x * pts[i].x;
			Y2 += pts[i].y * pts[i].y;
		}

		X /= pts.size();
		Y /= pts.size();
		XY /= pts.size();
		X2 /= pts.size();
		Y2 /= pts.size();

		a = -(XY - X * Y);

		T Bx = X2 - X * X;
		T By = Y2 - Y * Y;

		if (std::fabs(Bx) < std::fabs(By)) {
			b = By;
			std::swap(b, a);
		}
		else
			b = Bx;

		c = (a * X + b * Y);
	}

	Point<T> intercept(const Line<T>& l2)
	{
		Line<T> l1  = *this;
		double  det = l1.a * l2.b - l2.a * l1.b;

		if (std::fabs(det) < 1e-3)
			return Point<T>(INF, INF);
		else {
			double x = (l2.b * l1.c - l1.b * l2.c) / det;
			double y = (l1.a * l2.c - l2.a * l1.c) / det;

			return Point<T>(x, y);
		}
	}

	double getY(double x) { return (c - a * x) / b; }

	double dist(const Point<double>& p)
	{
		double den = sqrt((a * a + b * b));
		return std::fabs(a * p.x + b * p.y - c) / den;
	}
};

template <typename T>
struct Match
{
	Point<T> l_pos;
	Point<T> r_pos;
	Line<T>  l_line;
	Line<T>  r_line;

	Match() {}

	Match(const Point<T>& l, const Point<T>& r, const Line<T>& l_line,
	      const Line<T>& r_line)
	{
		(*this).l_pos  = Point<T>(l);
		(*this).r_pos  = Point<T>(r);
		(*this).l_line = l_line;
		(*this).r_line = r_line;
	}
};

template <typename T>
struct Ellipse
{
	T std_x;
	T std_y;
	T th;

	Ellipse() {}

	Ellipse(T std_x, T std_y, T th)
	{
		(*this).std_x = std_x;
		(*this).std_y = std_y;
		(*this).th    = th;
	}
};

template <typename T>
struct Landmark
{
	Ellipse<double> stdev;
	Point<T>      pos;
	int           n_obsv;

	Landmark() {}

	Landmark(const Point<T>& pos, const Ellipse<double>& stdev)
	{
		(*this).pos    = pos;
		(*this).stdev  = stdev;
		(*this).n_obsv = 1;
	}
};

template <typename T>
struct Particle
{
	int    id;
	double dy;
	double w;
	double cov;
	double dist;

	Particle() {}

	Particle(const int& id, const double& dy)
	{
		(*this).id = id;
		(*this).dy = dy;
		(*this).w  = 1;
	}
};

static std::vector<cv::Scalar>
    colors = {cv::Scalar(137, 137, 0),   cv::Scalar(0, 137, 137),
              cv::Scalar(137, 0, 137),   cv::Scalar(20, 165, 255),
              cv::Scalar(137, 137, 137), cv::Scalar(70, 0, 0),
              cv::Scalar(30, 20, 100),   cv::Scalar(10, 60, 200),
              cv::Scalar(4, 100, 40),    cv::Scalar(200, 200, 20),
              cv::Scalar(90, 170, 150),  cv::Scalar(150, 255, 255),
              cv::Scalar(2, 30, 60),     cv::Scalar(30, 39, 100),
              cv::Scalar(30, 50, 2),     cv::Scalar(200, 0, 30),
              cv::Scalar(255, 0, 0),     cv::Scalar(0, 255, 0),
              cv::Scalar(0, 50, 10)};

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

template <typename T1, typename T2>
Pose<T1> operator-(const Pose<T1>& p1, const Pose<T2>& p2)
{
	return Pose<T1>(p1.pos.x - p2.pos.x, p1.pos.y - p2.pos.y,
	                p1.theta - p2.theta);
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

template <typename T>
std::ostream& operator<<(std::ostream& o, const Particle<T>& p)
{
	o << "[id,dy,w,cov] = [" << p.id << "," << p.dy << "," << p.w << "," << p.cov
	  << "]" << std::endl;
	return o;
}
