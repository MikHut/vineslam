#include "../include/vineslam/map_io/elevation_map_parser.hpp"

namespace vineslam
{
ElevationMapParser::ElevationMapParser(const Parameters& params)
{
  file_path_ = params.elevation_map_input_file_;
}

bool ElevationMapParser::parseHeader(Parameters* params)
{
  std::ifstream xmlfile(file_path_);

  if (!xmlfile.is_open())
  {
    return false;
  }

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
    else if (tag == openTag((WIDTH)))
    {
      params->gridmap_width_ = getFloat(line);
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

  return true;
}

bool ElevationMapParser::parseFile(ElevationMap* elevation_map)
{
  std::ifstream xmlfile(file_path_);

  if (!xmlfile.is_open())
  {
    return false;
  }

  // Read map data, and fill the occupancy grid
  bool readingdata = true;
  int state = 0;
  float x = 0, y = 0, altitude;

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
        else if (tag == closeTag(DATA_))
        {
          readingdata = false;
        }
        break;
      case 1:
        if (tag == openTag(X_COORDINATE))
        {
          x = getFloat(line);
        }
        else if (tag == openTag(Y_COORDINATE))
        {
          y = getFloat(line);
        }
        else if (tag == openTag(ALTITUDE))
        {
          altitude = getFloat(line);
          (*elevation_map)(x, y) = altitude;
          state = 0;
        }
        break;
      default:
        readingdata = false;
        break;
    }
  }

  return true;
}

std::string ElevationMapParser::openTag(const std::string& m_tag)
{
  return '<' + m_tag + '>';
}

std::string ElevationMapParser::closeTag(const std::string& m_tag)
{
  return "</" + m_tag + '>';
}

std::string ElevationMapParser::getTag(const std::string& line)
{
  return line.substr(0, line.find('>') + 1);
}

float ElevationMapParser::getFloat(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto last_it = sub_str.find('<');

  std::string val_str = sub_str.substr(0, last_it);

  return std::stof(val_str);
}

int ElevationMapParser::getInt(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto last_it = sub_str.find('<');

  std::string val_str = sub_str.substr(0, last_it);

  return std::stoi(val_str);
}

std::string ElevationMapParser::getString(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto last_it = sub_str.find('<');

  return sub_str.substr(0, last_it);
}

}  // namespace vineslam