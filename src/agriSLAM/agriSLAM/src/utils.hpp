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

const float INF = 1.0e6;         // hypothetic infinit
const float PI  = 3.14159265359; // (radians)

// Parameters structure
struct Parameters
{
	double h_fov;     // Camera horizontal field of view (radians)
	double f_length;  // Camera focal lenght (pixels)
	int    width;     // Image width
	int    height;    // Image height
	double baseline;  // Stereo baseline between cameras (meters)
	double delta_D;   // Stereo matcher disparity error (pixels)
	double min_score; // Trunk detector minimum thresold

	std::string image_left;  // left image ROS topic
	std::string image_depth; // depth image ROS topic
	std::string odom_topic;  // odometry ROS topic
	std::string model;       // tflite model path
	std::string labels;      // detection labels path

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

// structures

template <typename T>
struct Point
{
	T x;
	T y;
	T z;

	Point() {}

	Point(const T& x, const T& y)
	{
		(*this).x = x;
		(*this).y = y;
	}

	Point(const T& x, const T& y, const T& z)
	{
		(*this).x = x;
		(*this).y = y;
		(*this).z = z;
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

	Eigen::VectorXd eig_2d()
	{
		Eigen::VectorXd vec(2, 1);
		vec << x, y;
		return vec;
	}

	Eigen::VectorXd eig_3d()
	{
		Eigen::VectorXd vec(3, 1);
		vec << x, y, z;
		return vec;
	}
};

template <typename T>
struct Pose
{
	Point<T> pos;
	T        roll;
	T        pitch;
	T        yaw;

	Pose() {}

	Pose(const T& x, const T& y, const double& theta)
	{
		(*this).pos.x = x;
		(*this).pos.y = y;
		(*this).yaw   = yaw;
	}

	Pose(const T& x, const T& y, const T& z, const T& roll, const T& pitch,
	     const T& yaw)
	{
		(*this).pos.x = x;
		(*this).pos.y = y;
		(*this).pos.z = z;
		(*this).roll  = roll;
		(*this).pitch = pitch;
		(*this).yaw   = yaw;
	}

	Eigen::VectorXd eig_2d()
	{
		Eigen::VectorXd vec(3, 1);
		vec << pos.x, pos.y, yaw;
		return vec;
	}

	Eigen::VectorXd eig_3d()
	{
		Eigen::VectorXd vec(6, 1);
		vec << pos.x, pos.y, pos.z, roll, pitch, yaw;
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
    cv::Scalar(137, 137, 0), cv::Scalar(0, 137, 137), cv::Scalar(137, 0, 137),
    cv::Scalar(20, 165, 255)};

// Overloading operators
template <typename T>
std::ostream& operator<<(std::ostream& o, const Point<T>& pt)
{
	o << "[x,y] = [" << pt.x << "," << pt.y << "]" << std::endl;
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
std::ostream& operator<<(std::ostream& o, const Ellipse<T>& e)
{
	o << "[dx,dy,dth] = [" << e.std_x << "," << e.std_y << "," << e.th << "]"
	  << std::endl;
	return o;
}

template <typename T1, typename T2>
Pose<T1> operator+(const Pose<T1>& p1, const Pose<T2>& p2)
{
	return Pose<T1>(p1.pos.x + p2.pos.x, p1.pos.y + p2.pos.y, p1.pos.z + p2.pos.z,
	                p1.roll + p2.roll, p1.pitch + p2.pitch, p1.yaw + p2.yaw);
}

template <typename T1, typename T2>
Pose<T1> operator-(const Pose<T1>& p1, const Pose<T2>& p2)
{
	return Pose<T1>(p1.pos.x - p2.pos.x, p1.pos.y - p2.pos.y, p1.pos.z - p2.pos.z,
	                p1.roll - p2.roll, p1.pitch - p2.pitch, p1.yaw - p2.yaw);
}
