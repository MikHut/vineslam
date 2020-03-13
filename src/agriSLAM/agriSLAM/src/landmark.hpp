#pragma once

#include <pose.hpp>
#include <landmark.hpp>
#include <iostream>

// Structure to represent the semantic information about
// each feature:
// - type of feature
// - description of the feature
// - dynamic or static
struct SemanticInfo
{
	SemanticInfo() {}
	SemanticInfo(const std::string& type, const std::string& description,
	             const int& character)
	{
		(*this).type        = type;
		(*this).description = description;
		(*this).character   = character;
	}

	std::string type;
	std::string description;
	int         character;
};

template <class T>
class Landmark
{
public:
	Landmark() {}
	// Class constructor
	// - initializes its pose, standard deviation and
	// - its sematic information
	Landmark(const Point<T>& pos, const Ellipse<T>& stdev,
	         const SemanticInfo& info)
	{
		(*this).pos   = pos;
		(*this).stdev = stdev;
		(*this).info  = info;
	}
	// Class constructor
	// - initializes its pose, standard deviation
	Landmark(const Point<T>& pos, const Ellipse<T>& stdev)
	{
		(*this).pos   = pos;
		(*this).stdev = stdev;
	}

  // Print semantic landmark information
	void print()
	{
		std::string c = (info.character == 0) ? "static" : "dynamic";

    std::cout << "Landmark " << std::endl;
    std::cout << "   type:        " << info.type << std::endl;
    std::cout << "   description: " << info.description << std::endl;
    std::cout << "   character:   " << c << std::endl;
    std::cout << "   position:    " << pos;
    std::cout << "   stdev:       " << stdev << std::endl;
	}

	Point<T>     pos;
	Ellipse<T>   stdev;
	SemanticInfo info;

private:
};
