#pragma once

#include <iostream>
#include <pose.hpp>

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

	// Initialize semantic information of feature to give
	// to the mapper class
	SemanticInfo(const int& label)
	{
		std::string type;
		std::string desc;
		int         ch;

		switch (label) {
		case 0:
			type = "Trunk";
			desc = "Vine trunk. A static landmark";
			ch   = 0;

			*this = SemanticInfo(type, desc, ch);
			break;
		case 1:
			type = "Leaf";
			desc = "Leaf from a vine trunk. A dynamic landmark";
			ch   = 1;

			*this = SemanticInfo(type, desc, ch);
			break;
		default:
			*this = SemanticInfo("Trunk", "Vine trunk", 0);
		}
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
	Landmark(const Point<T>& pos, const Ellipse<T>& stdev, const int& label)
	{
		(*this).pos   = pos;
		(*this).stdev = stdev;
		(*this).info  = SemanticInfo(label);
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
		std::cout << "   stdev:      [" << stdev.std_x << "," << stdev.std_y << "]"
		          << std::endl;
	}

	Point<T>     pos;
	Ellipse<T>   stdev;
	SemanticInfo info;

private:
};
