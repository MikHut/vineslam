#pragma once

#include <vector>

#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/math/Geodetic.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/lookup_edge.hpp>

namespace vineslam
{
// Vertex of a node
struct PlaceVertices
{
  float x_;
  float y_;

  double lon_;
  double lat_;
};

struct Vertex
{
  float x_;
  float y_;

  double lat_;
  double lon_;
};

// Graph node
struct Node
{
  uint32_t index_;
  Vertex center_;
  std::vector<PlaceVertices> rectangle_;
  std::vector<PlaceVertices> aligned_rectangle_;
  double rectangle_orientation_;

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
  TopologicalMap();

  // Initialize the topological map
  // - stores an occupancy map in the topological structure
  // - updates the rectangle orientations considering the map datum
  void init(OccupancyMap* input_map, const double& datum_head);

  // Compute the enu location of all the graph vertexes
  void polar2Enu(const double& datum_lat, const double& datum_lon, const double& datum_alt, const double& datum_head);

  // Compute the adjacent vertexes of a specific vertex
  void getAdjacentList(vertex_t v, std::vector<uint32_t>* adj_index);

  // Get active nodes
  void getActiveNodes(const Pose& robot_pose);

  // Deallocate nodes that are no longer active
  // Should also save their data into a file
  void deallocateNodes(const Pose& robot_pose);

  // Get the node where a point is or should be stored
  bool getNode(const Point& point, vertex_t& node);

  // Get the cell where a point is or should be stored
  void getCell(const Point& point, Cell* cell);

  // Instanciate a graph
  Graph map_;
  // Vector of vertexes
  std::vector<vertex_t> graph_vertexes_;
  // Vector of vertexes corresponding to the active nodes
  std::vector<vertex_t> active_nodes_vertexes_;
  // Flag to tell if the topological map was already initialized
  bool is_initialized_;

private:
};
}  // namespace vineslam
