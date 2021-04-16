#pragma once

#include <iostream>
#include <ctime>
#include <fstream>

#include <vineslam/mapping/elevation_map.hpp>
#include <vineslam/params.hpp>
#include <vineslam/map_io/map_writer.hpp>
#include <vineslam/map_io/elevation_map_writer.hpp>

namespace vineslam
{
class ElevationMapParser
{
public:
  ElevationMapParser(const Parameters& params);

  // Receives the occupancy grid map and writes it to a xml file
  bool parseHeader(Parameters *params);
  bool parseFile(ElevationMap* elevation_map);

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
  std::string file_path_;
};
}  // namespace vineslam