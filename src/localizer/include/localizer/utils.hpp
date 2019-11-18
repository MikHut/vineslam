#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

const float INF       = 1.0e6;         /* hypothetic infinit */
const float PI        = 3.14159265359; /* (radians) */
const float STD_XY    = 0.3;    /* position standard deviation (meters) */
const float STD_THETA = 0.1745; /* orientation standard deviation (radians) */

struct Parameters
{
	double h_fov;      /* Camera horizontal field of view */
	double v_fov;      /* Camera vertical field of view */
	double cam_height; /* Camera height (centimenters) */
	double min_score;  /* Minimum trunk detection probability */
	int    width;      /* Image width. */
	int    height;     /* Image height */
	int    scaler;     /* Convert meter to the unit of the map */

	std::string pose_topic;  /* pose ROS topic */
	std::string image_topic; /* image ROS topic */
	std::string map_file;    /* map file containing the landmarks position */
	std::string model;       /* tflite model path */
	std::string labels;      /* detection labels path */

	Parameters()
	{
		h_fov       = PI / 2;
		v_fov       = PI / 2;
		cam_height  = 100.0;
		width       = 1280;
		height      = 960;
		min_score   = 0.35;
		scaler      = 1;
		pose_topic  = "";
		image_topic = "";
		map_file    = "";
		model       = "";
		labels      = "";
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

		if (std::fabs(det) < 1e-3)
			return Point<T>(INF, INF);
		else {
			double x = (l2.b * l1.c - l1.b * l2.c) / det;
			double y = (l1.a * l2.c - l2.a * l1.c) / det;

			return Point<T>(x, y);
		}
	}
};

template <typename T>
struct Particle
{
	int      id;
	Point<T> pos;
	double   theta;
	double   weight;
	double   r_error;

	Particle() {}

	Particle(const int& id, const Point<T>& delta_p, const double& delta_th,
	         const double& weight)
	{
		(*this).id       = id;
		(*this).delta_p  = delta_p;
		(*this).delta_th = delta_th;
		(*this).weight   = weight;
		(*this).r_error  = 0.0;
	}
};

template <typename T>
struct Cell
{
	Point<T> index;
	int      score;

	Cell(){};

	Cell(const Point<T>& index)
	{
		(*this).index = index;
		score         = 0;
	}
};

template <typename T>
struct Grid
{
	int width;
	int height;

	std::vector<Cell<T>> cells;

	Grid() {}

	Grid(const int& width, const int& height)
	{
		(*this).width  = width;
		(*this).height = height;
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				Point<T> index(i, j - height / 2);
				Cell<T>  tmp(index);
				cells.push_back(tmp);
			}
		}
	}

	void clean() { cells.clear(); }

	int arrayIndex(Point<T> index)
	{
		for (size_t i = 0; i < cells.size(); i++) {
			if (cells[i].index.x == index.x && cells[i].index.y == index.y)
				return i;
		}

		return -1;
	}
};

template <typename T>
struct Landmark
{
	int      id;
	Point<T> world_pos;

	Landmark() {}

	Landmark(const int& id, const Point<T>& world_pos)
	{
		(*this).id        = id;
		(*this).world_pos = world_pos;
	}
};

static std::vector<cv::Scalar> colors = {
    cv::Scalar(137, 137, 0),   cv::Scalar(0, 137, 137),
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

template <typename T>
std::ostream& operator<<(std::ostream& o, const Particle<T>& p)
{
	o << "(" << p.id << ") - " << p.pos << "theta = " << p.theta
	  << "\nweight = " << p.weight << "\nr_error = " << p.r_error << std::endl;
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
	o << "id: " << l.id << "\n[x,y] = [" << l.world_pos.x << "," << l.world_pos.y
	  << "]" << std::endl;
	return o;
}
