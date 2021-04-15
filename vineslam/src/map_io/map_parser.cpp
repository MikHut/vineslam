#include "../../include/vineslam/map_io/map_parser.hpp"

namespace vineslam
{
MapParser::MapParser(const Parameters& params)
{
  file_path_ = params.map_input_file_;
}

void MapParser::parseHeader(Parameters* params)
{
  std::ifstream xmlfile(file_path_);
  bool readinginfo = true;

  // Read xml header
  std::string tmp;
  std::getline(xmlfile, tmp);

  // Read map info
  while (readinginfo)
  {
    // Read entire line
    std::string line;
    std::getline(xmlfile, line);

    // Extract tag
    std::string tag = getTag(line);
    tag.erase(std::remove_if(tag.begin(), tag.end(), isspace), tag.end());

    if (tag == openTag(INFO) || tag == openTag(ORIGIN))
    {
      continue;
    }
    else if (tag == openTag(X_COORDINATE))
    {
      params->gridmap_origin_x_ = getFloat(line);
    }
    else if (tag == openTag(Y_COORDINATE))
    {
      params->gridmap_origin_y_ = getFloat(line);
    }
    else if (tag == openTag(Z_COORDINATE))
    {
      params->gridmap_origin_z_ = getFloat(line);
    }
    else if (tag == openTag((WIDTH)))
    {
      params->gridmap_width_ = getFloat(line);
    }
    else if (tag == openTag(HEIGHT))
    {
      params->gridmap_height_ = getFloat(line);
    }
    else if (tag == openTag(LENGHT))
    {
      params->gridmap_lenght_ = getFloat(line);
    }
    else if (tag == openTag(RESOLUTION))
    {
      params->gridmap_resolution_ = getFloat(line);
      readinginfo = false;
    }
  }
}

void MapParser::parseFile(OccupancyMap* grid_map)
{
  std::ifstream xmlfile(file_path_);

  if (!xmlfile.is_open())
  {
    return;
  }

  // Read map data, and fill the occupancy grid
  bool readingdata = true;
  int state = 0;
  float x, y, z;
  int landmark_id;
  SemanticFeature semantic_feature;
  Corner corner;
  Planar planar;
  ImageFeature surf_feature;
  SemiPlane semi_plane;
  Point pt;

  // Read map data
  while (readingdata)
  {
    // Read entire line
    std::string line;
    std::getline(xmlfile, line);

    // Extract tag
    std::string tag = getTag(line);
    tag.erase(std::remove_if(tag.begin(), tag.end(), isspace), tag.end());

    switch (state)
    {
      case 0:
        if (tag == openTag(CELL))  // new cell
        {
          state = 1;
        }
        else if (tag == openTag(PLANES))  // read planes
        {
          state = 11;
        }
        else if (tag == closeTag(DATA_))
        {
          readingdata = false;
        }
        break;
      case 1:  // reading cell position
        if (tag == openTag(X_COORDINATE))
        {
          x = getFloat(line);
        }
        else if (tag == openTag(Y_COORDINATE))
        {
          y = getFloat(line);
        }
        else if (tag == openTag(Z_COORDINATE))
        {
          z = getFloat(line);
          state = 2;
        }
        break;
      case 2:  // check if we're going to read or not a new landmark
        if (tag == openTag(LTAG))
        {
          semantic_feature = SemanticFeature();
          state = 3;
        }
        else if (tag == closeTag(SEMANTICF))
          state = 4;
      case 3:  // reading a new landmark
        if (tag == openTag(ID_))
        {
          landmark_id = getInt(line);
        }
        else if (tag == openTag(X_COORDINATE))
        {
          semantic_feature.pos_.x_ = getFloat(line);
          semantic_feature.gauss_.mean_.x_ = semantic_feature.pos_.x_;
        }
        else if (tag == openTag(Y_COORDINATE))
        {
          semantic_feature.pos_.y_ = getFloat(line);
          semantic_feature.gauss_.mean_.y_ = semantic_feature.pos_.y_;
        }
        else if (tag == openTag(STDX))
        {
          semantic_feature.gauss_.stdev_.x_ = getFloat(line);
        }
        else if (tag == openTag(STDY))
        {
          semantic_feature.gauss_.stdev_.y_ = getFloat(line);
        }
        else if (tag == openTag(ANGLE))
        {
          semantic_feature.gauss_.theta_ = getFloat(line);
        }
        else if (tag == openTag(LABEL))
        {
          int c = getInt(line);
          semantic_feature.info_ = SemanticInfo(c);
        }
        else if (tag == closeTag(LTAG))
        {
          (*grid_map).insert(semantic_feature, landmark_id);
          state = 2;
        }
        break;
      case 4:  // check if we're going to read or not a new corner
        if (tag == openTag(CTAG))
        {
          corner = Corner();
          state = 5;
        }
        else if (tag == closeTag(CORNERF))
          state = 6;
        break;
      case 5:  // read a new corner
        if (tag == openTag(X_COORDINATE))
        {
          corner.pos_.x_ = getFloat(line);
        }
        else if (tag == openTag(Y_COORDINATE))
        {
          corner.pos_.y_ = getFloat(line);
        }
        else if (tag == openTag(Z_COORDINATE))
        {
          corner.pos_.z_ = getFloat(line);
        }
        else if (tag == openTag(PLANE))
        {
          corner.which_plane_ = getInt(line);
        }
        else if (tag == closeTag(CTAG))
        {
          (*grid_map).insert(corner);
          state = 4;
        }
        break;
      case 6:  // check if we're going to read or not a new planar
        if (tag == openTag(PTAG))
        {
          planar = Planar();
          state = 7;
        }
        else if (tag == closeTag(PLANARF))
          state = 8;
        break;
      case 7:  // read a new corner
        if (tag == openTag(X_COORDINATE))
        {
          planar.pos_.x_ = getFloat(line);
        }
        else if (tag == openTag(Y_COORDINATE))
        {
          planar.pos_.y_ = getFloat(line);
        }
        else if (tag == openTag(Z_COORDINATE))
        {
          planar.pos_.z_ = getFloat(line);
        }
        else if (tag == openTag(PLANE))
        {
          planar.which_plane_ = getInt(line);
        }
        else if (tag == closeTag(PTAG))
        {
          (*grid_map).insert(planar);
          state = 6;
        }
        break;
      case 8:  // check if we're going to read or not a new image feature
        if (tag == openTag(STAG))
        {
          surf_feature = ImageFeature();
          state = 9;
        }
        else if (tag == closeTag(SURFF))
          state = 0;
        break;
      case 9:  // reading a new image feature
        if (tag == openTag(X_COORDINATE))
        {
          surf_feature.pos_.x_ = getFloat(line);
        }
        else if (tag == openTag(Y_COORDINATE))
        {
          surf_feature.pos_.y_ = getFloat(line);
        }
        else if (tag == openTag(Z_COORDINATE))
        {
          surf_feature.pos_.z_ = getFloat(line);
        }
        else if (tag == openTag(U_))
        {
          surf_feature.u_ = getInt(line);
        }
        else if (tag == openTag(V_))
        {
          surf_feature.v_ = getInt(line);
        }
        else if (tag == openTag(RED))
        {
          surf_feature.r_ = getInt(line);
        }
        else if (tag == openTag(GREEN))
        {
          surf_feature.g_ = getInt(line);
        }
        else if (tag == openTag(BLUE))
        {
          surf_feature.b_ = getInt(line);
        }
        else if (tag == openTag(LAPLACIAN))
        {
          surf_feature.laplacian_ = getInt(line);
        }
        else if (tag == openTag(VALUE))
        {
          state = 10;
        }
        else if (tag == closeTag(STAG))
        {
          (*grid_map).insert(surf_feature);
          state = 8;
        }
        break;
      case 10:  // reading the values of an image feature descriptor
        if (tag == openTag(VALUE))
        {
          float s = getFloat(line);
          surf_feature.signature_.push_back(s);
          state = 9;
        }
        break;
      case 11:  // we will start to read a plane
        if (tag == openTag(PLANE))
        {
          semi_plane = SemiPlane();
          state = 12;
        }
        else if (tag == closeTag(PLANES))
          state = 0;
        break;
      case 12:  // we are reading the general arguments of a plane
        if (tag == openTag(COEF_A))
        {
          semi_plane.a_ = getFloat(line);
        }
        else if (tag == openTag(COEF_B))
        {
          semi_plane.b_ = getFloat(line);
        }
        else if (tag == openTag(COEF_C))
        {
          semi_plane.c_ = getFloat(line);
        }
        else if (tag == openTag(COEF_D))
        {
          semi_plane.d_ = getFloat(line);
        }
        else if (tag == openTag(CENTROID))
        {
          state = 13;
        }
        else if (tag == openTag(POINT))
        {
          pt = Point();
          state = 14;
        }
        else if (tag == openTag(EXTREMA))
        {
          pt = Point();
          state = 15;
        }
        else if (tag == closeTag(PLANE))
        {
          grid_map->planes_.push_back(semi_plane);
          state = 11;
        }
        break;
      case 13:  // reading the plane centroid
        if (tag == openTag(X_COORDINATE))
        {
          semi_plane.centroid_.x_ = getFloat(line);
        }
        else if (tag == openTag(Y_COORDINATE))
        {
          semi_plane.centroid_.y_ = getFloat(line);
        }
        else if (tag == openTag(Z_COORDINATE))
        {
          semi_plane.centroid_.z_ = getFloat(line);
        }
        else if (tag == closeTag(CENTROID))
        {
          state = 12;
        }
        break;
      case 14:  // reading the plane points
        if (tag == openTag(X_COORDINATE))
        {
          pt.x_ = getFloat(line);
        }
        else if (tag == openTag(Y_COORDINATE))
        {
          pt.y_ = getFloat(line);
        }
        else if (tag == openTag(Z_COORDINATE))
        {
          pt.z_ = getFloat(line);
        }
        else if (tag == closeTag(POINT))
        {
          semi_plane.points_.push_back(pt);
          state = 12;
        }
        break;
      case 15:  // reading the plane extremas
        if (tag == openTag(X_COORDINATE))
        {
          pt.x_ = getFloat(line);
        }
        else if (tag == openTag(Y_COORDINATE))
        {
          pt.y_ = getFloat(line);
        }
        else if (tag == openTag(Z_COORDINATE))
        {
          pt.z_ = getFloat(line);
        }
        else if (tag == closeTag(EXTREMA))
        {
          semi_plane.extremas_.push_back(pt);
          state = 12;
        }
        break;

      default:
        readingdata = false;
        break;
    }
  }
}

std::string MapParser::openTag(const std::string& m_tag)
{
  return '<' + m_tag + '>';
}

std::string MapParser::closeTag(const std::string& m_tag)
{
  return "</" + m_tag + '>';
}

std::string MapParser::getTag(const std::string& line)
{
  return line.substr(0, line.find('>') + 1);
}

float MapParser::getFloat(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto last_it = sub_str.find('<');

  std::string val_str = sub_str.substr(0, last_it);

  return std::stof(val_str);
}

int MapParser::getInt(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto last_it = sub_str.find('<');

  std::string val_str = sub_str.substr(0, last_it);

  return std::stoi(val_str);
}

std::string MapParser::getString(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto last_it = sub_str.find('<');

  return sub_str.substr(0, last_it);
}

}  // namespace vineslam