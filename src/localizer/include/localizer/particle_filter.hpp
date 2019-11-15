#pragma once

#include "utils.hpp"
#include <iostream>
#include <math.h>
#include <random>

class ParticleFilter
{
public:
	ParticleFilter(const Parameters& params);
	void init(const std::vector<Landmark<double>>& landmarks);
	void process(const std::vector<Point<double>>& land_poses);

	std::vector<Particle<double>> particles;

private:
	void predict(const std::vector<Point<double>>& land_poses);
	void updateWeights();
	void resample();

	double columnToTheta(const int& col)
	{
		return (-params.h_fov / params.width) * (params.width / 2 - col);
	}

	int thetaToColumn(const double& th)
	{
		return (th + params.h_fov / 2) * params.width / params.h_fov;
	}

	Parameters                    params;
	std::vector<double>           weights;
	std::vector<Landmark<double>> landmarks;
};
