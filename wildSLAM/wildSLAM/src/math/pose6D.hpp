#pragma once

#include <cmath>
#include <iostream>
#include <ellipse2D.hpp>

class pose6D
{
public:
	// Default constructor
	pose6D()
	{
		x     = 0.;
		y     = 0.;
		z     = 0.;
		roll  = 0.;
		pitch = 0.;
		yaw   = 0.;
	}

	// Construct with values
	pose6D(const float& x_, const float& y_, const float& z_, const float& roll_,
	       const float& pitch_, const float& yaw_)
	{
		x     = x_;
		y     = y_;
		z     = z_;
		roll  = roll_;
		pitch = pitch_;
		yaw   = yaw_;
	}

	// Construct with another pose6D
	pose6D(const pose6D& other)
	{
		x     = other.x;
		y     = other.y;
		z     = other.z;
		roll  = other.roll;
		pitch = other.pitch;
		yaw   = other.yaw;
	}

	// Assignment operator
	pose6D operator=(const pose6D& other) 
	{
		x = other.x;
		y = other.y;
		z = other.z;
		roll = other.roll;
		pitch = other.pitch;
		yaw = other.yaw;

    dist = other.dist;

    return *this;
	}

	// Addition operator
	pose6D operator+(const pose6D& other) const
	{
		pose6D result(*this);
		result.x += other.x;
		result.y += other.y;
		result.z += other.z;
		result.roll += other.roll;
		result.pitch += other.pitch;
		result.yaw += other.yaw;

		return result;
	}

	// Subtraction operator
	pose6D operator-(const pose6D& other) const
	{
		pose6D result(*this);
		result.x -= other.x;
		result.y -= other.y;
		result.z -= other.z;
		result.roll -= other.roll;
		result.pitch -= other.pitch;
		result.yaw -= other.yaw;

		return result;
	}

  // Set gaussian noise characterizing the robot pose
  void setGaussian(const ellipse2D& other)
  {
    dist = other;
  }

	// 3D euclidean distance
	float distance(const pose6D& other) const
	{
		float dist_x = x - other.x;
		float dist_y = y - other.y;
		float dist_z = z - other.z;
		return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
	}

	// 2D euclidean distance
	float distanceXY(const pose6D& other) const
	{
		float dist_x = x - other.x;
		float dist_y = y - other.y;
		return sqrt(dist_x * dist_x + dist_y * dist_y);
	}

	// Cartesian coordinates
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;

  // Gaussian uncertainty
  ellipse2D dist;

private:
};

// stdout operator
std::ostream& operator<<(std::ostream& out, pose6D const& p)
{
	return out << '(' << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.roll << ' '
	           << p.pitch << ' ' << p.yaw << ')';
}

