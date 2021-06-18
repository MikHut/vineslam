#pragma once

#include <vector>

#include <vineslam/math/Point.hpp>
#include <vineslam/mapping/occupancy_map.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/lookup_edge.hpp>

namespace vineslam
{
// Vertex of a node
struct PlaceVertices
{
  uint16_t x_;
  uint16_t y_;
};

// Graph node
struct Node
{
  uint32_t index_;
  Point center_;
  std::vector<PlaceVertices> rectangle_;

  OccupancyMap* map_;
};

// Graph edge
struct Edge
{
  int8_t i_;
};

// Some typedefs for simplicity
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, Node, Edge> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<Graph>::edge_descriptor edge_t;
typedef boost::graph_traits<Graph>::adjacency_iterator AdjacencyIterator;

class TopologicalMap
{
public:
  TopologicalMap() = default;

  // Instanciate a graph
  Graph map_;

  std::vector<vertex_t> graph_vertexes_;

private:
};
}  // namespace vineslam