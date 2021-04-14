#pragma once

#include <iostream>
#include <ctime>
#include <fstream>

#include <vineslam/mapping/elevation_map.hpp>
#include <vineslam/params.hpp>
#include <vineslam/map_io/map_writer.hpp>

#define ALTITUDE "altitude"

namespace vineslam
{
class ElevationMapWriter
{
public:
  ElevationMapWriter(const Parameters& params);

  // Receives the elevation grid map and writes it to a xml file
  void writeToFile(ElevationMap* elevation_map);

private:
  // Opens a tag with a specified value
  static std::string open(const std::string& tag);
  // Closes a tag with a specified value
  static std::string close(const std::string& tag);

  std::string file_path_;
};
}  // namespace vineslam
