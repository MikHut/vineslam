#include "../../include/vineslam/map_io/map_writer.hpp"

namespace vineslam
{
MapWriter::MapWriter(const Parameters& params, const std::time_t& timestamp)
{
  file_path_ = params.map_output_folder_ + "map_" + std::to_string(timestamp) + ".xml";
}

void MapWriter::writeToFile(OccupancyMap* grid_map, const Parameters& params)
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
  xmlfile << TAB << TAB << open(X_COORDINATE) << grid_map->origin_.x_ << close(X_COORDINATE) << ENDL;
  xmlfile << TAB << TAB << open(Y_COORDINATE) << grid_map->origin_.y_ << close(Y_COORDINATE) << ENDL;
  xmlfile << TAB << TAB << open(Z_COORDINATE) << grid_map->origin_.z_ << close(Z_COORDINATE) << ENDL;
  xmlfile << TAB << close(ORIGIN) << ENDL;
  xmlfile << TAB << open(WIDTH) << grid_map->width_ << close(WIDTH) << ENDL;
  xmlfile << TAB << open(HEIGHT) << grid_map->height_ << close(HEIGHT) << ENDL;
  xmlfile << TAB << open(LENGHT) << grid_map->lenght_ << close(LENGHT) << ENDL;
  xmlfile << TAB << open(RESOLUTION) << grid_map->resolution_ << close(RESOLUTION) << ENDL;
  xmlfile << close(INFO) << ENDL << ENDL;

  // -- Map data
  xmlfile << open(DATA_) << ENDL;
  for (auto layer : *grid_map)
  {
    // Compute map layer bounds
    float xmin = layer.second.origin_.x_;
    float xmax = xmin + layer.second.width_;
    float ymin = layer.second.origin_.y_;
    float ymax = xmin + layer.second.lenght_;
    float z = static_cast<float>(layer.first) * grid_map->resolution_z_ + grid_map->origin_.z_;
    for (float i = xmin; i < xmax - grid_map->resolution_;)
    {
      for (float j = ymin; j < ymax - grid_map->resolution_;)
      {
        // Check if there is any feature in the current cell
        if (layer.second(i, j).data == nullptr)
        {
          j += grid_map->resolution_;
          continue;
        }

        std::map<int, SemanticFeature>* l_landmarks = layer.second(i, j).data->landmarks_;
        std::vector<ImageFeature>* l_image_features = layer.second(i, j).data->surf_features_;
        std::vector<Corner>* l_corners = layer.second(i, j).data->corner_features_;
        std::vector<Planar>* l_planars = layer.second(i, j).data->planar_features_;

        // Check if cell is empty: we have to check first is the pointers to the arrays are valid :-)
        bool is_empty = true;
        if (l_landmarks != nullptr)
        {
          if (!l_landmarks->empty())
          {
            is_empty = false;
          }
        }
        if (l_image_features != nullptr)
        {
          if (!l_image_features->empty())
          {
            is_empty = false;
          }
        }
        if (l_corners != nullptr)
        {
          if (!l_corners->empty())
          {
            is_empty = false;
          }
        }
        if (l_planars != nullptr)
        {
          if (!l_planars->empty())
          {
            is_empty = false;
          }
        }

        // Check if there is any feature in the current cell
        if (is_empty)
        {
          j += grid_map->resolution_;
          continue;
        }

        xmlfile << TAB << open(CELL) << ENDL;
        xmlfile << TAB << TAB << open(X_COORDINATE) << i << close(X_COORDINATE) << ENDL;
        xmlfile << TAB << TAB << open(Y_COORDINATE) << j << close(Y_COORDINATE) << ENDL;
        xmlfile << TAB << TAB << open(Z_COORDINATE) << z << close(Z_COORDINATE) << ENDL;

        // High level semantic features
        xmlfile << TAB << TAB << open(SEMANTICF) << ENDL;
        if (l_landmarks == nullptr)
        {
        }
        else
        {
          for (const auto& landmark : *l_landmarks)
          {
            xmlfile << TAB << TAB << TAB << open(LTAG) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(ID_) << landmark.first << close(ID_) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(X_COORDINATE) << landmark.second.pos_.x_ << close(X_COORDINATE)
                    << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(Y_COORDINATE) << landmark.second.pos_.y_ << close(Y_COORDINATE)
                    << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(STDX) << landmark.second.gauss_.stdev_.x_ << close(STDX)
                    << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(STDY) << landmark.second.gauss_.stdev_.y_ << close(STDY)
                    << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(ANGLE) << landmark.second.gauss_.theta_ << close(ANGLE) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(LABEL) << landmark.second.info_.character_ << close(LABEL)
                    << ENDL;
            xmlfile << TAB << TAB << TAB << close(LTAG) << ENDL;
          }
        }
        xmlfile << TAB << TAB << close(SEMANTICF) << ENDL;

        // Medium level corner features
        xmlfile << TAB << TAB << open(CORNERF) << ENDL;
        if (l_corners == nullptr)
        {
        }
        else
        {
          for (const auto& corner : *l_corners)
          {
            xmlfile << TAB << TAB << TAB << open(CTAG) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(X_COORDINATE) << corner.pos_.x_ << close(X_COORDINATE) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(Y_COORDINATE) << corner.pos_.y_ << close(Y_COORDINATE) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(Z_COORDINATE) << corner.pos_.z_ << close(Z_COORDINATE) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(PLANE) << corner.which_plane_ << close(PLANE) << ENDL;
            xmlfile << TAB << TAB << TAB << close(CTAG) << ENDL;
          }
        }
        xmlfile << TAB << TAB << close(CORNERF) << ENDL;

        // Medium level planar features
        xmlfile << TAB << TAB << open(PLANARF) << ENDL;
        if (l_planars == nullptr)
        {
        }
        else
        {
          for (const auto& planar : *l_planars)
          {
            xmlfile << TAB << TAB << TAB << open(PTAG) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(X_COORDINATE) << planar.pos_.x_ << close(X_COORDINATE) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(Y_COORDINATE) << planar.pos_.y_ << close(Y_COORDINATE) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(Z_COORDINATE) << planar.pos_.z_ << close(Z_COORDINATE) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(PLANE) << planar.which_plane_ << close(PLANE) << ENDL;
            xmlfile << TAB << TAB << TAB << close(PTAG) << ENDL;
          }
        }
        xmlfile << TAB << TAB << close(PLANARF) << ENDL;

        // Low level image features
        xmlfile << TAB << TAB << open(SURFF) << ENDL;
        if (l_image_features == nullptr)
        {
        }
        else
        {
          for (const auto& surf_feature : *l_image_features)
          {
            xmlfile << TAB << TAB << TAB << open(STAG) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(X_COORDINATE) << surf_feature.pos_.x_ << close(X_COORDINATE)
                    << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(Y_COORDINATE) << surf_feature.pos_.y_ << close(Y_COORDINATE)
                    << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(Z_COORDINATE) << surf_feature.pos_.z_ << close(Z_COORDINATE)
                    << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(U_) << surf_feature.u_ << close(U_) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(V_) << surf_feature.v_ << close(V_) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(RED) << surf_feature.r_ << close(RED) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(GREEN) << surf_feature.g_ << close(GREEN) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(BLUE) << surf_feature.b_ << close(BLUE) << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(LAPLACIAN) << surf_feature.laplacian_ << close(LAPLACIAN)
                    << ENDL;
            xmlfile << TAB << TAB << TAB << TAB << open(SIGNATURE) << ENDL;
            for (float m_signature : surf_feature.signature_)
            {
              xmlfile << TAB << TAB << TAB << TAB << TAB << open(VALUE) << m_signature << close(VALUE) << ENDL;
            }
            xmlfile << TAB << TAB << TAB << TAB << close(SIGNATURE) << ENDL;
            xmlfile << TAB << TAB << TAB << close(STAG) << ENDL;
          }
        }
        xmlfile << TAB << TAB << close(SURFF) << ENDL;

        j += grid_map->resolution_;
        xmlfile << TAB << close(CELL) << ENDL;
      }
      i += grid_map->resolution_;
    }
  }

  // Medium level semiplanes
  xmlfile << TAB << open(PLANES) << ENDL;
  for (const auto& plane : grid_map->planes_)
  {
    xmlfile << TAB << TAB << open(PLANE) << ENDL;

    xmlfile << TAB << TAB << TAB << open(COEF_A) << plane.a_ << close(COEF_A) << ENDL;
    xmlfile << TAB << TAB << TAB << open(COEF_B) << plane.b_ << close(COEF_B) << ENDL;
    xmlfile << TAB << TAB << TAB << open(COEF_C) << plane.c_ << close(COEF_C) << ENDL;
    xmlfile << TAB << TAB << TAB << open(COEF_D) << plane.d_ << close(COEF_D) << ENDL;

    xmlfile << TAB << TAB << TAB << open(CENTROID) << ENDL;
    xmlfile << TAB << TAB << TAB << TAB << open(X_COORDINATE) << plane.centroid_.x_ << close(X_COORDINATE) << ENDL;
    xmlfile << TAB << TAB << TAB << TAB << open(Y_COORDINATE) << plane.centroid_.y_ << close(Y_COORDINATE) << ENDL;
    xmlfile << TAB << TAB << TAB << TAB << open(Z_COORDINATE) << plane.centroid_.z_ << close(Z_COORDINATE) << ENDL;
    xmlfile << TAB << TAB << TAB << close(CENTROID) << ENDL;

    xmlfile << TAB << TAB << TAB << open(POINTS_) << ENDL;
    for (const auto& pt : plane.points_)
    {
      xmlfile << TAB << TAB << TAB << TAB << open(POINT) << ENDL;
      xmlfile << TAB << TAB << TAB << TAB << TAB << open(X_COORDINATE) << pt.x_ << close(X_COORDINATE) << ENDL;
      xmlfile << TAB << TAB << TAB << TAB << TAB << open(Y_COORDINATE) << pt.y_ << close(Y_COORDINATE) << ENDL;
      xmlfile << TAB << TAB << TAB << TAB << TAB << open(Z_COORDINATE) << pt.z_ << close(Z_COORDINATE) << ENDL;
      xmlfile << TAB << TAB << TAB << TAB << close(POINT) << ENDL;
    }
    xmlfile << TAB << TAB << TAB << close(POINTS_) << ENDL;

    xmlfile << TAB << TAB << TAB << open(EXTREMAS) << ENDL;
    for (const auto& pt : plane.extremas_)
    {
      xmlfile << TAB << TAB << TAB << TAB << open(EXTREMA) << ENDL;
      xmlfile << TAB << TAB << TAB << TAB << TAB << open(X_COORDINATE) << pt.x_ << close(X_COORDINATE) << ENDL;
      xmlfile << TAB << TAB << TAB << TAB << TAB << open(Y_COORDINATE) << pt.y_ << close(Y_COORDINATE) << ENDL;
      xmlfile << TAB << TAB << TAB << TAB << TAB << open(Z_COORDINATE) << pt.z_ << close(Z_COORDINATE) << ENDL;
      xmlfile << TAB << TAB << TAB << TAB << close(EXTREMA) << ENDL;
    }
    xmlfile << TAB << TAB << TAB << close(EXTREMAS) << ENDL;

    xmlfile << TAB << TAB << close(PLANE) << ENDL;
  }
  xmlfile << TAB << close(PLANES) << ENDL;

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

}  // namespace vineslam
