#pragma once

#include <iostream>
#include <ctime>
#include <fstream>

#include <vineslam/mapping/occupancy_map.hpp>

// ----- TAGS ----- //
// -- General
#define HEADER "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
#define ENDL "\n"
#define TAB "    "
// -- Map info
#define DATUM "datum"
#define LATITUDE "latitude"
#define LONGITUDE "longitude"
#define ALTITUDE "altitude"
#define HEADING "heading"
#define INFO "info"
#define ORIGIN "origin"
#define WIDTH "width"
#define HEIGHT "height"
#define LENGHT "lenght"
#define RESOLUTION "resolution"
// -- Map data
#define VALUE "value"
#define CELL "cell"
#define DATA_ "data"
#define X_COORDINATE "x"
#define Y_COORDINATE "y"
#define Z_COORDINATE "z"
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
// ----- Planars
#define PLANARF "planar_features"
#define PTAG "p"
// ----- SURF
#define SURFF "surf_features"
#define STAG "s"
#define U_ "u"
#define V_ "v"
#define RED "r"
#define GREEN "g"
#define BLUE "b"
#define LAPLACIAN "laplacian"
#define SIGNATURE "signature"
// ----- PLANES
#define PLANES "planes"
#define POINTS_ "points"
#define POINT "point"
#define CENTROID "centroid"
#define EXTREMAS "extremas"
#define EXTREMA "extrema"
#define COEF_A "a"
#define COEF_B "b"
#define COEF_C "c"
#define COEF_D "d"

namespace vineslam
{
class MapWriter
{
public:
  // Class constructor - loads the file name
  MapWriter(const Parameters& params, const std::time_t& timestamp);
  MapWriter(const Parameters& params, const uint32_t& id);

  // Receives the occupancy grid map and writes it to a xml file
  void writeToFile(OccupancyMap* grid_map, const Parameters& params);

private:
  // Opens a tag with a specified value
  static std::string open(const std::string& tag);
  // Closes a tag with a specified value
  static std::string close(const std::string& tag);

  // Input parameters
  std::string file_path_;
};
}  // namespace vineslam
