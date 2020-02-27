#pragma once

#include <utils.hpp>
#include <landmark.hpp>

#include <iostream>
#include <map>

class Localizer
{
public:
	// Class constructor
	Localizer();

	// Global function that handles all the localization process
	void process(const Point<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>&             depths,
	             const std::map<int, Landmark<double>>& map);

private:
};
