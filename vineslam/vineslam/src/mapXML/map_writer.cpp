#include "map_writer.hpp"

namespace vineslam
{

MapWriter::MapWriter(const Parameters& params)
{
  // Read input parameters
  file_path = params.map_output_folder;

  std::time_t t = std::time(0);
  file_path += "map_" + std::to_string(t) + ".xml";
}

void MapWriter::writeToFile(OccupancyMap grid_map)
{
  // Create file
  std::ofstream xmlfile;
  xmlfile.open(file_path);

  // --------------------------------------------------------------------------------
  // --------- Write to file
  // --------------------------------------------------------------------------------
  // -- XML header
  xmlfile << HEADER << ENDL << ENDL;

  // -- Grid map details
  xmlfile << open(INFO) << ENDL;
  xmlfile << TAB << open(ORIGIN) << ENDL;
  xmlfile << TAB << TAB << open(X_) << grid_map.origin.x << close(X_) << ENDL;
  xmlfile << TAB << TAB << open(Y_) << grid_map.origin.y << close(Y_) << ENDL;
  xmlfile << TAB << close(ORIGIN) << ENDL;
  xmlfile << TAB << open(WIDTH) << grid_map.width << close(WIDTH) << ENDL;
  xmlfile << TAB << open(HEIGHT) << grid_map.height << close(HEIGHT) << ENDL;
  xmlfile << TAB << open(RESOLUTION) << grid_map.resolution << close(RESOLUTION)
          << ENDL;
  xmlfile << TAB << open(METRIC) << grid_map.metric << close(METRIC) << ENDL;
  xmlfile << close(INFO) << ENDL << ENDL;

  // -- Map data
  xmlfile << open(DATA_) << ENDL;
  int xmin = static_cast<int>(grid_map.origin.x / grid_map.resolution);
  int xmax = static_cast<int>(static_cast<float>(xmin) +
                              grid_map.width / grid_map.resolution - 1);
  int ymin = static_cast<int>(grid_map.origin.y / grid_map.resolution);
  int ymax = static_cast<int>(static_cast<float>(ymin) +
                              grid_map.height / grid_map.resolution - 1);
  for (int i = xmin; i < xmax; i++) {
    for (int j = ymin; j < ymax; j++) {
      // Check if there is any feature in the current cell
      if (grid_map(i, j).landmarks.empty() &&
          grid_map(i, j).corner_features.empty() &&
          grid_map(i, j).surf_features.empty())
        continue;

      xmlfile << TAB << open(CELL) << ENDL;
      xmlfile << TAB << TAB << open(X_) << i << close(X_) << ENDL;
      xmlfile << TAB << TAB << open(Y_) << j << close(Y_) << ENDL;

      // High level semantic features
      xmlfile << TAB << TAB << open(SEMANTICF) << ENDL;
      for (const auto& landmark : grid_map(i, j).landmarks) {
        xmlfile << TAB << TAB << TAB << open(LTAG) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(ID_) << landmark.first
                << close(ID_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(X_) << landmark.second.pos.x
                << close(X_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(Y_) << landmark.second.pos.y
                << close(Y_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(STDX)
                << landmark.second.gauss.stdev.x << close(STDX) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(STDY)
                << landmark.second.gauss.stdev.y << close(STDY) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(ANGLE)
                << landmark.second.gauss.theta << close(ANGLE) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(LABEL)
                << landmark.second.info.character << close(LABEL) << ENDL;
        xmlfile << TAB << TAB << TAB << close(LTAG) << ENDL;
      }
      xmlfile << TAB << TAB << close(SEMANTICF) << ENDL;

      // Medium level corner features
      xmlfile << TAB << TAB << open(CORNERF) << ENDL;
      for (const auto& corner : grid_map(i, j).corner_features) {
        xmlfile << TAB << TAB << TAB << open(CTAG) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(X_) << corner.pos.x << close(X_)
                << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(Y_) << corner.pos.y << close(Y_)
                << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(Z_) << corner.pos.z << close(Z_)
                << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(PLANE) << corner.which_plane
                << close(PLANE) << ENDL;
        xmlfile << TAB << TAB << TAB << close(CTAG) << ENDL;
      }
      xmlfile << TAB << TAB << close(CORNERF) << ENDL;

      // Low level image features
      xmlfile << TAB << TAB << open(SURFF) << ENDL;
      for (const auto& surf_feature : grid_map(i, j).surf_features) {
        xmlfile << TAB << TAB << TAB << open(STAG) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(X_) << surf_feature.pos.x
                << close(X_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(Y_) << surf_feature.pos.y
                << close(Y_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(Z_) << surf_feature.pos.z
                << close(Z_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(U_) << surf_feature.u
                << close(U_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(V_) << surf_feature.v
                << close(V_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(R_) << surf_feature.r
                << close(R_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(G_) << surf_feature.g
                << close(G_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(B_) << surf_feature.b
                << close(B_) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(LAPLACIAN)
                << surf_feature.laplacian << close(LAPLACIAN) << ENDL;
        xmlfile << TAB << TAB << TAB << TAB << open(SIGNATURE) << ENDL;
        for (float m_signature : surf_feature.signature) {
          xmlfile << TAB << TAB << TAB << TAB << TAB << open(VALUE) << m_signature
                  << close(VALUE) << ENDL;
        }
        xmlfile << TAB << TAB << TAB << TAB << close(SIGNATURE) << ENDL;
        xmlfile << TAB << TAB << TAB << close(STAG) << ENDL;
      }
      xmlfile << TAB << TAB << close(SURFF) << ENDL;
    }
  }
  xmlfile << TAB << close(CELL) << ENDL;
  xmlfile << close(DATA_) << ENDL << ENDL;
  // --------------------------------------------------------------------------------

  // Save file
  xmlfile.close();
}

std::string MapWriter::open(const std::string& tag)
{
  std::string out = "<" + tag + ">";
  return out;
}
std::string MapWriter::close(const std::string& tag)
{
  std::string out = "</" + tag + ">";
  return out;
}

} // namespace vineslam
