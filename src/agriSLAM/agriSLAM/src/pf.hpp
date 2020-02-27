#pragma once

#include <utils.hpp>

#include <iostream>

// Struct that represents a single particle with
// - identification number
// - 6-DOF pose
// - weight
struct Particle
{
	Particle() {}
	Particle(const int id, const Pose<double>& pose, const double& w);
	int    id;
  Pose<double> pose;
	double w;
};

class PF
{
public:
	// Class constructor
	PF();

	// Global function that handles all the particle filter processes
	void process(const Point<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>& depths);


private:
	// Prediction step
	void predict(const Point<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>& depths);
	// Update ste
	void update();
	// Resampling over all particles
	void resample();
};
