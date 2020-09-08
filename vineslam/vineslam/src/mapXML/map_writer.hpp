#pragma once

#include <iostream>
#include <ctime>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "../occupancy_map.hpp"

// ----- TAGS ----- //
// -- General
#define HEADER "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
#define ENDL "\n"
#define TAB "    "
// -- Map info
#define INFO "info"
#define ORIGIN "origin"
#define WIDTH "width"
#define HEIGHT "height"
#define RESOLUTION "resolution"
#define METRIC "metric"
// -- Map data
#define VALUE "value"
#define CELL "cell"
#define DATA_ "data"
#define X_ "x"
#define Y_ "y"
#define Z_ "z"
// ----- Semantic
#define SEMANTICF "semantic_features"
#define ID_ "id"
#define LTAG "l"
#define STDX "std_x"
#define STDY "std_y"
#define ANGLE "angle"
#define LABEL "label"
// ----- Corners
#define CORNERF "corner_features"
#define CTAG "c"
#define PLANE "plane"
// ----- SURF
#define SURFF "surf_features"
#define STAG "s"
#define U_ "u"
#define V_ "v"
#define R_ "r"
#define G_ "g"
#define B_ "b"
#define LAPLACIAN "laplacian"
#define SIGNATURE "signature"

namespace vineslam
{
class MapWriter
{
public:
  // Class constructor - loads the file name
  explicit MapWriter(const std::string& config_file);

  // Receives the occupancy grid map and writes it to a xml file
  void writeToFile(OccupancyMap grid_map);

private:
  // Opens a tag with a specified value
  static std::string open(const std::string& tag);
  // Closes a tag with a specified value
  static std::string close(const std::string& tag);

  // Input parameters
  std::string file_path;
};
} // namespace vineslam
