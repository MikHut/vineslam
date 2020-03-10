#pragma once

// Include class objects
#include <landmark.hpp>
#include <utils.hpp>

#include <iostream>
#include <map>
#include <random>

// Struct that represents a single particle with
// - identification number
// - 6-DOF pose
// - weight
struct Particle
{
	Particle() {}
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

static std::ostream& operator<<(std::ostream& o, const Particle& p)
{
  o << "Particle " << p.id << ":\n" << p.pose << p.w << "\n\n";
	return o;
}

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

  // Export the array of particles
  void getParticles(std::vector<Particle>& in);

private:
	// Prediction step - particles inovation using a motion model
	void predict(const Pose<double>& odom);
	// Correction step
	// - calculate a local map for each particle
	// - update particle weight using the difference between the local and
	//   the global map
	// - normalize the weights
	void correct(const std::vector<double>& bearings,
	             const std::vector<double>&             depths,
	             const std::map<int, Landmark<double>>& map);
	// Resampling over all particles
	void resample();
  
	// Auxiliar function that normalizes an angle in the [-pi,pi] range
	double normalizeAngle(const double& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}

	// Previous odometry control
	Pose<double> p_odom;
	// Array of particles
	std::vector<Particle> particles;
};
