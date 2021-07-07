#pragma once

#include <iostream>
#include <ctime>
#include <fstream>

#include <vineslam/mapping/topological_map.hpp>
#include <vineslam/params.hpp>
#include <vineslam/map_io/map_writer.hpp>

namespace vineslam
{
class TopologicalMapParser
{
public:
  TopologicalMapParser(const Parameters& params);

  // Receives the occupancy grid map and writes it to a xml file
  bool parseFile(TopologicalMap* topological_map);

  // Tags to parse the file
  const std::string s_header{ "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" };
  const std::string s_map_file{ "map" };
  const std::string s_tab{ "   " };
  const std::string s_endl{ "\n" };
  const std::string s_vertex{ "vertex" };
  const std::string s_index{ "index" };
  const std::string s_circle{ "circle" };
  const std::string s_x{ "x" };
  const std::string s_y{ "y" };
  const std::string s_lat{ "lat" };
  const std::string s_lon{ "lon" };
  const std::string s_radius{ "radius" };
  const std::string s_rectangle{ "rectangle" };
  const std::string s_x1{ "x1" };
  const std::string s_y1{ "y1" };
  const std::string s_x2{ "x2" };
  const std::string s_y2{ "y2" };
  const std::string s_lat_1{ "lat_1" };
  const std::string s_lon_1{ "lon_1" };
  const std::string s_lat_2{ "lat_2" };
  const std::string s_lon_2{ "lon_2" };
  const std::string s_edges{ "edges" };
  const std::string s_edge{ "edge" };
  const std::string s_v1{ "v1" };
  const std::string s_v2{ "v2" };

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
