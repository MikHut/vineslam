#include "../include/vineslam/map_io/elevation_map_writer.hpp"

namespace vineslam
{
ElevationMapWriter::ElevationMapWriter(const Parameters& params, const std::time_t& timestamp)
{
  file_path_ = params.map_output_folder_ + "elevation_map_" + std::to_string(timestamp) + ".xml";
}

void ElevationMapWriter::writeToFile(ElevationMap* elevation_map, const Parameters& params)
{
  // Create file
  std::ofstream xmlfile;
  xmlfile.open(file_path_);

  // --------------------------------------------------------------------------------
  // --------- Write to file
  // --------------------------------------------------------------------------------
  // -- XML header
  xmlfile << HEADER << ENDL << ENDL;

  // -- Grid map details
  xmlfile << open(INFO) << ENDL;
  xmlfile << TAB << open(DATUM) << ENDL;
  xmlfile << TAB << TAB << open(LATITUDE) << params.map_datum_lat_ << close(LATITUDE) << ENDL;
  xmlfile << TAB << TAB << open(LONGITUDE) << params.map_datum_long_ << close(LONGITUDE) << ENDL;
  xmlfile << TAB << TAB << open(ALTITUDE) << params.map_datum_alt_ << close(ALTITUDE) << ENDL;
  xmlfile << TAB << TAB << open(HEADING) << params.map_datum_head_ << close(HEADING) << ENDL;
  xmlfile << TAB << close(DATUM) << ENDL;
  xmlfile << TAB << open(ORIGIN) << ENDL;
  xmlfile << TAB << TAB << open(X_COORDINATE) << elevation_map->origin_.x_ << close(X_COORDINATE) << ENDL;
  xmlfile << TAB << TAB << open(Y_COORDINATE) << elevation_map->origin_.y_ << close(Y_COORDINATE) << ENDL;
  xmlfile << TAB << close(ORIGIN) << ENDL;
  xmlfile << TAB << open(WIDTH) << elevation_map->width_ << close(WIDTH) << ENDL;
  xmlfile << TAB << open(LENGHT) << elevation_map->lenght_ << close(LENGHT) << ENDL;
  xmlfile << TAB << open(RESOLUTION) << elevation_map->resolution_ << close(RESOLUTION) << ENDL;
  xmlfile << close(INFO) << ENDL << ENDL;

  // -- Map data
  xmlfile << open(DATA_) << ENDL;
  // Compute map layer bounds
  float xmin = elevation_map->origin_.x_;
  float xmax = xmin + elevation_map->width_;
  float ymin = elevation_map->origin_.y_;
  float ymax = ymin + elevation_map->lenght_;
  for (float i = xmin; i < xmax - elevation_map->resolution_;)
  {
    for (float j = ymin; j < ymax - elevation_map->resolution_;)
    {
      if ((*elevation_map)(i, j) == 0)
      {
        j += elevation_map->resolution_;
        continue;
      }
      
      xmlfile << TAB << open(CELL) << ENDL;
      xmlfile << TAB << TAB << open(X_COORDINATE) << i << close(X_COORDINATE) << ENDL;
      xmlfile << TAB << TAB << open(Y_COORDINATE) << j << close(Y_COORDINATE) << ENDL;
      xmlfile << TAB << TAB << open(ALTITUDE) << (*elevation_map)(i, j) << close(ALTITUDE) << ENDL;
      xmlfile << TAB << close(CELL) << ENDL;

      j += elevation_map->resolution_;
    }
    i += elevation_map->resolution_;
  }
  xmlfile << close(DATA_) << ENDL;

  // Save file
  xmlfile.close();
}


std::string ElevationMapWriter::open(const std::string& tag)
{
  std::string out = "<" + tag + ">";
  return out;
}
std::string ElevationMapWriter::close(const std::string& tag)
{
  std::string out = "</" + tag + ">";
  return out;
}

}  // namespace vineslam
