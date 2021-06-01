#include "../include/vineslam/map_io/topological_map_parser.hpp"

namespace vineslam
{

TopologicalMapParser::TopologicalMapParser(const Parameters& params)
{
  file_path_ = params.topological_map_input_file_;
}

bool TopologicalMapParser::parseFile(TopologicalMap *topological_map)
{
/*  // Create input file
  std::ifstream xmlfile(file_path_);
  if (!xmlfile.is_open())
  {
    ROS_INFO("Input file not found.");
    return;
  }

  // Declare necessary variables
  Vertex v;
  uint32_t v1, v2;
  std::vector<Vertex> vertexes;
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
        if (tag == openTag(s_map_file))
        {
          map_file = getString(line);
        }
        else if (tag == openTag(s_vertex))
        {
          state = 1;
          v = Vertex();
          v.rectangle.resize(2);
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
          v.index = getInt(line);
        }
        else if (tag == openTag(s_x))
        {
          v.circle.x = getFloat(line);
        }
        else if (tag == openTag(s_y))
        {
          v.circle.y = getFloat(line);
        }
        else if (tag == openTag(s_radius))
        {
          v.circle.radius = getFloat(line);
        }
        else if (tag == openTag(s_x1))
        {
          v.rectangle[0].x = getFloat(line);
        }
        else if (tag == openTag(s_y1))
        {
          v.rectangle[0].y = getFloat(line);
        }
        else if (tag == openTag(s_x2))
        {
          v.rectangle[1].x = getFloat(line);
        }
        else if (tag == openTag(s_y2))
        {
          v.rectangle[1].y = getFloat(line);
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
    vertex_t u = boost::add_vertex(v, map);
    graph_vertex.push_back(u);
  }

  // Insert edges into the graph
  Edge edge;
  edge.i = 1;
  for (const auto& e : edges)
  {
    if (!boost::edge(graph_vertex[e.first], graph_vertex[e.second], map).second)
    {
      boost::add_edge(graph_vertex[e.first], graph_vertex[e.second], edge, map);
      boost::add_edge(graph_vertex[e.first], graph_vertex[e.second], edge, map);
    }
  }*/
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