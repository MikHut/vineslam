#pragma once

// Object members
#include <landmark.hpp>
#include <math/point3D.hpp>
#include <math/ellipse2D.hpp>

// ROS, std, Eigen
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <yaml-cpp/yaml.h>

#define PI 3.14159265359

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KF
{
public:
	KF(){};
	// Class constructor
	// - Receives the initial state and the parameters
	// - initializes the covariance matrix
	KF(const VectorXd& X0, const VectorXd& s, const VectorXd& g,
	   const VectorXd& z, const std::string& config_path);
	// Function that processes all the Kalman Filter routines
	void process(const VectorXd& s, const VectorXd& g, const VectorXd& z);
	// Function that outputs the current state of the Kalman Filter
	point3D getState() const;
	// Function that outputs the current standard deviation of the
	// Kalman Filter
	ellipse2D getStdev() const;

	MatrixXd P;

private:
	// State vector and KF matrices
	VectorXd X0;
	VectorXd X;
	MatrixXd K;
	MatrixXd R;

	// Input parameters
	float baseline;
	float delta_d;
	float fx;

	// Function that implements the prediction step of the Kalman Filter
	void predict();
	// Function that implements the update step of the Kalman Filter
	void correct(const VectorXd& s, const VectorXd& z);
	// Function that calculates the current observations covariance matrix
	void computeR(const VectorXd& s, const VectorXd& g, const VectorXd& z);

	// Auxiliar function that normalizes an angle in the [-pi,pi] range
	float normalizeAngle(const float& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}

	// Calculates the disparity error using the disparity noise model
	float dispError(const float& depth)
	{
		return pow(depth, 2) / (baseline * fx) * delta_d;
	}
};
