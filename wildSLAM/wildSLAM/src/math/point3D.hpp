#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>

class point3D
{
public:
	// Default constructor
	point3D()
	{
		x = 0.;
		y = 0.;
		z = 0.;
	}

	// Construct with values
	point3D(const float& x_, const float& y_, const float& z_)
	{
		x = x_;
		y = y_;
		z = z_;
	}

	// Construct with another point3D
	point3D(const point3D& other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
	}

	// Assignment operator
	point3D operator=(const point3D& other)
	{
		x = other.x;
		y = other.y;
		z = other.z;

		return *this;
	}

	// Addition operator
	point3D operator+(const point3D& other) const
	{
		point3D result(*this);
		result.x += other.x;
		result.y += other.y;
		result.z += other.z;

		return result;
	}

	// Subtraction operator
	point3D operator-(const point3D& other) const
	{
		point3D result(*this);
		result.x -= other.x;
		result.y -= other.y;
		result.z -= other.z;

		return result;
	}

	// 3D euclidean distance
	float distance(const point3D& other) const
	{
		float dist_x = x - other.x;
		float dist_y = y - other.y;
		float dist_z = z - other.z;
		return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
	}

	// 2D euclidean distance
	float distanceXY(const point3D& other) const
	{
		float dist_x = x - other.x;
		float dist_y = y - other.y;
		return sqrt(dist_x * dist_x + dist_y * dist_y);
	}

	// Convert point3D to Eigen 3D vector
	Eigen::VectorXd toEig2D()
	{
		Eigen::VectorXd vec(2, 1);
		vec << x, y;
		return vec;
	}

	// Convert point3D to Eigen 3D vector
	Eigen::VectorXd toEig3D()
	{
		Eigen::VectorXd vec(3, 1);
		vec << x, y, z;
		return vec;
	}

	// Cartesian coordinates
	float x;
	float y;
	float z;

private:
};

// stdout operator
static std::ostream& operator<<(std::ostream& out, point3D const& p)
{
	return out << '(' << p.x << ' ' << p.y << ' ' << p.z << ")\n";
}
