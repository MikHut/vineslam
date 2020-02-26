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
	double f_length;  /* Camera focal lenght (pixels) */
	int    width;     /* Image width */
	int    height;    /* Image height */
	double baseline;  /* Stereo baseline between cameras (meters) */
	double delta_D;   /* Stereo matcher disparity error (pixels) */
	double min_score; /* Trunk detector minimum thresold */

	std::string image_left;  /* left image ROS topic */
	std::string image_depth; /* depth image ROS topic */
	std::string odom_topic;  /* odometry ROS topic */
	std::string model;       /* tflite model path */
	std::string labels;      /* detection labels path */

	Parameters()
	{
		f_length  = 0.1;
		width     = 1280;
		height    = 960;
		min_score = 0.5;
		delta_D   = 0.2;

		model       = "";
		labels      = "";
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

// Array of cv colors
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

// Overloading operators

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
std::ostream& operator<<(std::ostream& o, const Ellipse<T>& e)
{
	o << "[dx,dy,dth] = [" << e.std_x << "," << e.std_y << "," << e.th << "]"
	  << std::endl;
	return o;
}
