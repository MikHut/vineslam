#pragma once

#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <vector>

const float INF = 1.0e6;         /* hypothetic infinit */
const float PI  = 3.14159265359; /* (radians) */

struct Parameters
{
	double h_fov;           /* Camera horizontal field of view (radians) */
	double v_fov;           /* Camera vertical field of view (radians) */
	double cam_height;      /* Camera height (centimenters) */
	int    width;           /* Image width. */
	int    height;          /* Image height */
	int    match_box;       /* Search box diagonal size */
	int    filter_window;   /* Dimension of the window of the robot pose filter */
	int    mapper_inc;      /* Increment between frames to use in the mapper */
	double min_score;       /* Minimum trunk detection probability */
	double vineyard_height; /* Vineyard height in centimeters */
	int    max_stdev;       /* Maximum standard deviation of trunk world position
	                           estimation */

	std::string pose_topic;  /* pose ROS topic */
	std::string image_topic; /* image ROS topic */
	std::string model;       /* tflite model path */
	std::string labels;      /* detection labels path */
	std::string prediction;  /* type of predictor - 'average' or 'histogram' */

	Parameters()
	{
		h_fov           = PI / 2;
		v_fov           = PI / 4;
		cam_height      = 100.0;
		width           = 1280;
		height          = 960;
		match_box       = 10;
		filter_window   = 5;
		mapper_inc      = 100;
		min_score       = 0.5;
		max_stdev       = 1000;
		vineyard_height = 150;
		model           = "";
		labels          = "";
		pose_topic      = "";
		image_topic     = "";
		prediction      = "average";
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
	int                    id;
	Point<double>          stdev;
	Point<T>               world_pos;
	std::vector<Point<T>>  estimations;
	std::vector<Point<T>>  image_pos;
	std::vector<Cell<int>> cells;
	std::vector<int>       ptr;

	Landmark() {}

	Landmark(const int& id, const Point<T>& image_pos)
	{
		(*this).id        = id;
		(*this).image_pos = std::vector<Point<T>>(1, image_pos);
	}

	void standardDev()
	{
		Point<double> mean = world_pos;
		Point<double> var  = Point<double>(0.0, 0.0);

		for (size_t i = 0; i < estimations.size(); i++) {
			var.x += (estimations[i].x - mean.x) * (estimations[i].x - mean.x);
			var.y += (estimations[i].y - mean.y) * (estimations[i].y - mean.y);
		}

		stdev.x = sqrt(var.x / estimations.size());
		stdev.y = sqrt(var.y / estimations.size());
	}

	Point<T> maxScore()
	{
		Point<T> index;
		int      max = 0;
		for (size_t i = 0; i < cells.size(); i++) {
			if (cells[i].score > max) {
				max   = cells[i].score;
				index = Point<T>((T)cells[i].index.x, (T)cells[i].index.y);
			}
		}
		return index;
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
