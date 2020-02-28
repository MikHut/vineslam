#pragma once

// Include class objects
#include <landmark.hpp>
#include <utils.hpp>

#include <iostream>
#include <map>

// Struct that represents a single particle with
// - identification number
// - 6-DOF pose
// - weight
struct Particle
{
	Particle();
	Particle(const int& id, const Pose<double>& pose, const double& w)
	{
		(*this).id   = id;
		(*this).pose = pose;
		(*this).w    = w;
	}
	int          id;
	Pose<double> pose;
	double       w;
};

class PF
{
public:
	// Class constructor
	// - initializes the total set of particles
	PF(const int& n_particles, const Pose<double>& initial_pose);

	// Global function that handles all the particle filter processes
	void process(const Pose<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>&             depths,
	             const std::map<int, Landmark<double>>& map);


private:
	// Prediction step - particles inovation using a motion model
	void predict(const Pose<double>& odom);
	// Correction step
	// - calculate a local map for each particle
	// - update particle weight using the difference between the local and
	//   the global map
	// - normalize the weights
	void correct(const Pose<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>&             depths,
	             const std::map<int, Landmark<double>>& map);
	// Resampling over all particles
	void resample();

	// Previous odometry control
	Pose<double> p_odom;
	// Array of particles
	std::vector<Particle> particles;
};
