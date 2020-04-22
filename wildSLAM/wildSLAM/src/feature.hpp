#pragma once

#include <iostream>

class Feature
{
public:
	Feature() {}
	// Class constructor
	// - initializes its image position and feature type
	Feature(const int& u, const int& v, const std::string& type)
	{
		(*this).u    = u;
		(*this).v    = v;
		(*this).type = type;
	}
	// Print semantic landmark information
	void print()
	{
		std::cout << "Feature " << std::endl;
		std::cout << "   type:        " << type << std::endl;
		std::cout << "   position:   [" << u << "," << v << "]" << std::endl;
	}

	int         u;
	int         v;
	std::string type;

private:
};

