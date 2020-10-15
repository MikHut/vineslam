#include "map_parser.hpp"

namespace vineslam
{

MapParser::MapParser(const Parameters& params)
{
  // Read input parameters
  file_path = params.map_input_file;
}

void MapParser::parseFile(MapLayer& grid_map)
{
  std::ifstream xmlfile(file_path);
  bool          readinginfo = true;

  // Read xml header
  std::string tmp;
  std::getline(xmlfile, tmp);

  // Read map info
  while (readinginfo) {
    // Read entire line
    std::string line;
    std::getline(xmlfile, line);

    // Extract tag
    std::string tag = getTag(line);
    tag.erase(std::remove_if(tag.begin(), tag.end(), isspace), tag.end());

    if (tag == openTag(INFO) || tag == openTag(ORIGIN)) {
      continue;
    } else if (tag == openTag(X_)) {
      grid_map.origin.x = getFloat(line);
    } else if (tag == openTag(Y_)) {
      grid_map.origin.y = getFloat(line);
    } else if (tag == openTag((WIDTH))) {
      grid_map.width = getFloat(line);
    } else if (tag == openTag(HEIGHT)) {
      grid_map.lenght = getFloat(line);
    } else if (tag == openTag(RESOLUTION)) {
      grid_map.resolution = getFloat(line);
    } else if (tag == openTag(METRIC)) {
      grid_map.metric = getString(line);
      readinginfo     = false;
    }
  }

  bool            readingdata = true;
  int             state       = 0;
  int             x, y;
  int             landmark_id;
  SemanticFeature m_semantic_feature;
  Corner          m_corner;
  ImageFeature    m_surf_feature;
  // Read map data
  while (readingdata) {
    // Read entire line
    std::string line;
    std::getline(xmlfile, line);

    // Extract tag
    std::string tag = getTag(line);
    tag.erase(std::remove_if(tag.begin(), tag.end(), isspace), tag.end());

    switch (state) {
      case 0: // new cell
        if (tag == openTag(CELL)) {
          state = 1;
        } else if (tag == closeTag(DATA_)) {
          readingdata = false;
        }
        break;
      case 1: // reading cell position
        if (tag == openTag(X_)) {
          x = getInt(line);
        } else if (tag == openTag(Y_)) {
          y     = getInt(line);
          state = 2;
        }
        break;
      case 2: // check if we're going to read or not a new landmark
        if (tag == openTag(LTAG)) {
          m_semantic_feature = SemanticFeature();
          state              = 3;
        } else if (tag == closeTag(SEMANTICF))
          state = 4;
      case 3: // reading a new landmark
        if (tag == openTag(ID_)) {
          landmark_id = getInt(line);
        } else if (tag == openTag(X_)) {
          m_semantic_feature.pos.x        = getFloat(line);
          m_semantic_feature.gauss.mean.x = m_semantic_feature.pos.x;
        } else if (tag == openTag(Y_)) {
          m_semantic_feature.pos.y        = getFloat(line);
          m_semantic_feature.gauss.mean.y = m_semantic_feature.pos.y;
        } else if (tag == openTag(STDX)) {
          m_semantic_feature.gauss.stdev.x = getFloat(line);
        } else if (tag == openTag(STDY)) {
          m_semantic_feature.gauss.stdev.y = getFloat(line);
        } else if (tag == openTag(ANGLE)) {
          m_semantic_feature.gauss.theta = getFloat(line);
        } else if (tag == openTag(LABEL)) {
          int c                   = getInt(line);
          m_semantic_feature.info = SemanticInfo(c);
        } else if (tag == closeTag(LTAG)) {
          grid_map(x, y).insert(landmark_id, m_semantic_feature);
          state = 2;
        }
        break;
      case 4: // check if we're going to read or not a new corner
        if (tag == openTag(CTAG)) {
          m_corner = Corner();
          state    = 5;
        } else if (tag == closeTag(CORNERF))
          state = 6;
        break;
      case 5: // read a new corner
        if (tag == openTag(X_)) {
          m_corner.pos.x = getFloat(line);
        } else if (tag == openTag(Y_)) {
          m_corner.pos.y = getFloat(line);
        } else if (tag == openTag(Z_)) {
          m_corner.pos.z = getFloat(line);
        } else if (tag == openTag(PLANE)) {
          m_corner.which_plane = getInt(line);
        } else if (tag == closeTag(CTAG)) {
          grid_map(x, y).insert(m_corner);
          state = 4;
        }
        break;
      case 6: // check if we're going to read or not a new image feature
        if (tag == openTag(STAG)) {
          m_surf_feature = ImageFeature();
          state          = 7;
        } else if (tag == closeTag(SURFF))
          state = 0;
        break;
      case 7: // reading a new image feature
        if (tag == openTag(X_)) {
          m_surf_feature.pos.x = getFloat(line);
        } else if (tag == openTag(Y_)) {
          m_surf_feature.pos.y = getFloat(line);
        } else if (tag == openTag(Z_)) {
          m_surf_feature.pos.z = getFloat(line);
        } else if (tag == openTag(U_)) {
          m_surf_feature.u = getInt(line);
        } else if (tag == openTag(V_)) {
          m_surf_feature.v = getInt(line);
        } else if (tag == openTag(R_)) {
          m_surf_feature.r = getInt(line);
        } else if (tag == openTag(G_)) {
          m_surf_feature.g = getInt(line);
        } else if (tag == openTag(B_)) {
          m_surf_feature.b = getInt(line);
        } else if (tag == openTag(LAPLACIAN)) {
          m_surf_feature.laplacian = getInt(line);
        } else if (tag == openTag(VALUE)) {
          state = 8;
        } else if (tag == closeTag(STAG)) {
          state = 6;
        }
        break;
      case 8: // reading the values of an image feature descriptor
        if (tag == openTag(VALUE)) {
          float s = getFloat(line);
          m_surf_feature.signature.push_back(s);
          state = 7;
        }
        break;
      default:
        readingdata = false;
        break;
    }
  }

  printMap(grid_map);
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
  auto        last_it = sub_str.find('<');

  std::string val_str = sub_str.substr(0, last_it);

  return std::stof(val_str);
}

int MapParser::getInt(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto        last_it = sub_str.find('<');

  std::string val_str = sub_str.substr(0, last_it);

  return std::stoi(val_str);
}

std::string MapParser::getString(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto        last_it = sub_str.find('<');

  return sub_str.substr(0, last_it);
}

void MapParser::printMap(MapLayer grid_map)
{
  std::cout << "-----------------------------------------------" << std::endl;
  std::cout << "----- Map info " << std::endl;
  std::cout << "-----------------------------------------------" << std::endl
            << std::endl;

  std::cout << "  [X,Y]: [" << grid_map.origin.x << "," << grid_map.origin.y << "]"
            << std::endl;
  std::cout << "  width: [" << grid_map.width << std::endl;
  std::cout << "  height: [" << grid_map.lenght << std::endl;
  std::cout << "  resolution: [" << grid_map.resolution << std::endl;
  std::cout << "  metric: [" << grid_map.metric << std::endl << std::endl;

  std::cout << "-----------------------------------------------" << std::endl;
  std::cout << "----- Map data " << std::endl;
  std::cout << "-----------------------------------------------" << std::endl
            << std::endl;

  int xmin = static_cast<int>(grid_map.origin.x / grid_map.resolution);
  int xmax = static_cast<int>(static_cast<float>(xmin) +
                              grid_map.width / grid_map.resolution - 1);
  int ymin = static_cast<int>(grid_map.origin.y / grid_map.resolution);
  int ymax = static_cast<int>(static_cast<float>(ymin) +
                              grid_map.lenght / grid_map.resolution - 1);
  for (int i = xmin; i < xmax; i++) {
    for (int j = ymin; j < ymax; j++) {
      // Check if there is any feature in the current cell
      if (grid_map(i, j).landmarks.empty() &&
          grid_map(i, j).corner_features.empty() &&
          grid_map(i, j).surf_features.empty())
        continue;

      std::cout << "CELL (" << i << "," << j << ")" << std::endl;
      for (const auto& landmark : grid_map(i, j).landmarks) {
        std::cout << "Landmark" << landmark.first << ": " << std::endl;
        std::cout << "   pos = " << landmark.second.pos;
        std::cout << "   gaussian mean " << landmark.second.gauss.mean;
        std::cout << "   gaussian stdev " << landmark.second.gauss.stdev;
        std::cout << "   gaussian angle " << landmark.second.gauss.theta
                  << std::endl;
        std::cout << "   info = " << landmark.second.info.type << " ; "
                  << landmark.second.info.description << std::endl;
      }

      for (const auto& corner : grid_map(i, j).corner_features) {
        std::cout << "Corner:" << std::endl;
        std::cout << "   pos = " << corner.pos;
        std::cout << "   plane = " << corner.which_plane << std::endl;
      }

      for (const auto& surf_feature : grid_map(i, j).surf_features) {
        std::cout << "Image feature:" << std::endl;
        std::cout << "   [u,v] = [" << surf_feature.u << "," << surf_feature.v << "]"
                  << std::endl;
        std::cout << "   pos = " << surf_feature.pos;
        std::cout << "   [r,g,b] = [" << surf_feature.r << "," << surf_feature.g
                  << "," << surf_feature.b << "]" << std::endl;
        std::cout << "   laplacian = " << surf_feature.laplacian << std::endl;
        std::cout << "   signature: " << std::endl;
        for (float f : surf_feature.signature) {
          std::cout << f << " ";
        }
        std::cout << std::endl;
      }
    }
  }
}

} // namespace vineslam