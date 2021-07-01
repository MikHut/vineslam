#include "../include/vineslam/map_io/topological_map_parser.hpp"

namespace vineslam
{
TopologicalMapParser::TopologicalMapParser(const Parameters& params)
{
  file_path_ = params.topological_map_input_file_;
}

bool TopologicalMapParser::parseFile(TopologicalMap* topological_map)
{
  // Create input file
  std::ifstream xmlfile(file_path_);
  if (!xmlfile.is_open())
  {
    std::cout << "Input file not found." << std::endl;
    return false;
  }

  // Declare necessary variables
  Node v;
  uint32_t v1, v2;
  std::vector<Node> vertexes;
  std::multimap<uint32_t, uint32_t> edges;
  bool readingdata = true;
  int state = 0;

  // Parse loop
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
      case 0:  // new vertex
        if (tag == openTag(s_vertex))
        {
          state = 1;
          v = Node();
          v.rectangle_.resize(2);
        }
        else if (tag == openTag(s_edge))
        {
          v1 = 0;
          v2 = 0;
          state = 2;
        }
        else if (tag == closeTag(s_edges))
        {
          readingdata = false;
        }
        break;
      case 1:
        if (tag == openTag(s_index))
        {
          v.index_ = getInt(line);
        }
        else if (tag == openTag(s_lat))
        {
          v.center_.lat_ = getFloat(line);
        }
        else if (tag == openTag(s_lon))
        {
          v.center_.lon_ = getFloat(line);
        }
        else if (tag == openTag(s_lat_1))
        {
          v.rectangle_[0].lat_ = getFloat(line);
        }
        else if (tag == openTag(s_lon_1))
        {
          v.rectangle_[0].lon_ = getFloat(line);
        }
        else if (tag == openTag(s_lat_2))
        {
          v.rectangle_[1].lat_ = getFloat(line);
        }
        else if (tag == openTag(s_lon_2))
        {
          v.rectangle_[1].lon_ = getFloat(line);
        }
        else if (tag == closeTag(s_vertex))
        {
          vertexes.push_back(v);
          state = 0;
        }

        break;
      case 2:
        if (tag == openTag(s_v1))
        {
          v1 = getInt(line);
        }
        else if (tag == openTag(s_v2))
        {
          v2 = getInt(line);
          edges.insert(std::pair<uint32_t, uint32_t>(v1, v2));
        }
        else if (tag == closeTag(s_edge))
        {
          edges.insert(std::pair<uint32_t, uint32_t>(v1, v2));
          state = 0;
        }
        break;
    }
  }

  // Insert vertexes into the graph
  for (const auto& v : vertexes)
  {
    vertex_t u = boost::add_vertex(v, topological_map->map_);
    topological_map->graph_vertexes_.push_back(u);
  }

  // Insert edges into the graph
  Edge edge;
  edge.i_ = 1;
  for (const auto& e : edges)
  {
    if (!boost::edge(topological_map->graph_vertexes_[e.first], topological_map->graph_vertexes_[e.second],
                     topological_map->map_)
             .second)
    {
      boost::add_edge(topological_map->graph_vertexes_[e.first], topological_map->graph_vertexes_[e.second], edge,
                      topological_map->map_);
      boost::add_edge(topological_map->graph_vertexes_[e.first], topological_map->graph_vertexes_[e.second], edge,
                      topological_map->map_);
    }
  }

  return true;
}

// ---------------------------------------------------------------
// ----- Parser aux functions
// ---------------------------------------------------------------

std::string TopologicalMapParser::openTag(const std::string& m_tag)
{
  return '<' + m_tag + '>';
}

std::string TopologicalMapParser::closeTag(const std::string& m_tag)
{
  return "</" + m_tag + '>';
}

std::string TopologicalMapParser::getTag(const std::string& line)
{
  return line.substr(0, line.find('>') + 1);
}

float TopologicalMapParser::getFloat(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto last_it = sub_str.find('<');

  std::string val_str = sub_str.substr(0, last_it);

  return std::stof(val_str);
}

int TopologicalMapParser::getInt(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto last_it = sub_str.find('<');

  std::string val_str = sub_str.substr(0, last_it);

  return std::stoi(val_str);
}

std::string TopologicalMapParser::getString(const std::string& line)
{
  std::string sub_str = line.substr(line.find('>') + 1);
  auto last_it = sub_str.find('<');

  return sub_str.substr(0, last_it);
}
}  // namespace vineslam
