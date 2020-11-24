#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <utility>

#include "map_writer.hpp"
#include "../mapping/occupancy_map.hpp"
#include "../feature/semantic.hpp"
#include "../feature/three_dimensional.hpp"
#include "../feature/visual.hpp"

namespace vineslam
{
class MapParser
{
public:
  // Class constructor - loads the file name
  explicit MapParser(Parameters  params);

  // Receives the occupancy grid map and writes it to a xml file
  void parseFile(OccupancyMap* grid_map);

private:
  // Build a xml tag (open) from a string
  static std::string openTag(const std::string& m_tag);
  // Build a xml tag (close) from a string
  static std::string closeTag(const std::string& m_tag);
  // Reads a tag
  static std::string getTag(const std::string& line);
  // Reads a float inside a tag
  static float getFloat(const std::string& line);
  // Reads an int inside a tag
  static int getInt(const std::string& line);
  // Reads a string inside a tag
  static std::string getString(const std::string& line);

  // Input parameters
  Parameters params_;
};
}  // namespace vineslam