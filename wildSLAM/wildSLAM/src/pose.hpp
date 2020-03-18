#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <landmark.hpp>
#include <vector>

const float INF = 1.0e6;         // hypothetic infinit
const float PI  = 3.14159265359; // (radians)

// Structure to represent a 2d or 3d point
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
		(*this).z = pt.z;
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

// Structure to represent a Gaussian using an Ellipse with:
// - x and y standard deviations
// - orientation of the ellipse
template <typename T>
struct Ellipse
{
	T std_x;
	T std_y;
	T th;

	Ellipse() {}

	Ellipse(const T& std_x, const T& std_y, const T& th)
	{
		(*this).std_x = std_x;
		(*this).std_y = std_y;
		(*this).th    = th;
	}
};

// Structure to represent a robot pose considering
// - 6-dof
// - gaussian noise characterization
template <typename T>
struct Pose
{
	// 6-dof robot pose
	Point<T> pos;
	T        roll;
	T        pitch;
	T        yaw;

	// Pose gaussian noise characterization
	Ellipse<T> gaussian;

	Pose() {}

	Pose(const T& x, const T& y, const double& yaw)
	{
		(*this).pos.x = x;
		(*this).pos.y = y;
		(*this).yaw   = yaw;
	}

	Pose(const T& x, const T& y, const double& yaw, const Ellipse<T>& gaussian)
	{
		(*this).pos.x    = x;
		(*this).pos.y    = y;
		(*this).yaw      = yaw;
		(*this).gaussian = gaussian;
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

	Pose(const T& x, const T& y, const T& z, const T& roll, const T& pitch,
	     const T& yaw, const Ellipse<T>& gaussian)
	{
		(*this).pos.x    = x;
		(*this).pos.y    = y;
		(*this).pos.z    = z;
		(*this).roll     = roll;
		(*this).pitch    = pitch;
		(*this).yaw      = yaw;
		(*this).gaussian = gaussian;
	}

	Pose(const std::vector<Pose<T>>& poses)
	{
		Pose<T>  mean = Pose<T>(0, 0, 0, 0, 0, 0);
		Point<T> std  = Point<T>(0, 0, 0);
		double   th   = 0;

		// Calculate mean of all the robot poses
		for (size_t i = 0; i < poses.size(); i++) {
			mean.pos.x += (T)poses[i].pos.x;
			mean.pos.y += (T)poses[i].pos.y;
			mean.pos.z += (T)poses[i].pos.z;
			mean.roll += (T)poses[i].roll;
			mean.pitch += (T)poses[i].pitch;
			mean.yaw += (T)poses[i].yaw;
		}
		mean.pos.x /= (T)poses.size();
		mean.pos.y /= (T)poses.size();
		mean.pos.z /= (T)poses.size();
		mean.roll /= (T)poses.size();
		mean.pitch /= (T)poses.size();
		mean.yaw /= (T)poses.size();

		// Calculate standard deviation of all the robot positions
		for (size_t i = 0; i < poses.size(); i++) {
			std.x += pow(poses[i].pos.x - mean.pos.x, 2);
			std.y += pow(poses[i].pos.y - mean.pos.y, 2);
		}
		std.x = sqrt(std.x / (T)poses.size());
		std.y = sqrt(std.y / (T)poses.size());
		th    = tan(mean.yaw);

		// Save pose and gaussian distribution
		(*this).pos.x = mean.pos.x;
		(*this).pos.y = mean.pos.y;
		(*this).pos.z = mean.pos.z;
		(*this).roll  = mean.roll;
		(*this).pitch = mean.pitch;
		(*this).yaw   = mean.yaw;

		(*this).gaussian = Ellipse<T>(std.x, std.y, th);
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

// Overloading operators
template <typename T>
std::ostream& operator<<(std::ostream& o, const Point<T>& pt)
{
	o << "[x,y,z] = [" << pt.x << "," << pt.y << "," << pt.z << "]" << std::endl;
	return o;
}

template <typename T1, typename T2>
Point<T1> operator-(const Point<T1>& p1, const Point<T2>& p2)
{
	return Point<T1>(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

template <typename T>
std::ostream& operator<<(std::ostream& o, const Pose<T>& p)
{
	o << "[x,y,z,roll,pitch,yaw] = [" << p.pos.x << "," << p.pos.y << ","
	  << p.pos.z << "," << p.roll << "," << p.pitch << "," << p.yaw << "]"
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
