#pragma once

#include <vector>

#include <vineslam/map_io/map_parser.hpp>
#include <vineslam/feature/semantic.hpp>
#include <vineslam/feature/visual.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/math/Geodetic.hpp>
#include <vineslam/params.hpp>

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
  bool has_file_;

  OccupancyMap* grid_map_{ nullptr };
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
  TopologicalMap(const Parameters& params);

  // Initialize the topological map
  // - stores an occupancy map in the topological structure
  void init(OccupancyMap* input_map, const double& heading);

  // Compute the enu location of all the graph vertexes
  void polar2Enu(const double& datum_lat, const double& datum_lon, const double& datum_alt, const double& datum_head);

  // Compute the adjacent vertexes of a specific vertex
  void getAdjacentList(vertex_t v, std::vector<uint32_t>* adj_index);

  // Get active nodes
  void getActiveNodes(const Pose& robot_pose);

  // Allocate memory for a occupancy map of a node
  void allocateNodeMap(const vertex_t& node);

  // Deallocate nodes that are no longer active
  // Should also save their data into a file
  void deallocateNodes(const Pose& robot_pose);

  // Get the node where a point is or should be stored
  bool getNode(const Point& point, vertex_t& node);

  // Get the cell where a point is or should be stored
  bool getCell(Point& point, Cell& cell, vertex_t& node, bool read_only = true);

  // Routines to obtain the features on a given location
  std::vector<Planar> getPlanars(const float& x, const float& y, const float& z);
  std::vector<Corner> getCorners(const float& x, const float& y, const float& z);
  std::map<int, SemanticFeature> getSemanticFeatures(const float& x, const float& y);
  std::vector<ImageFeature> getImageFeatures(const float& x, const float& y, const float& z);

  // Routines to obtain the all the features of a given type in a map
  std::vector<Planar> getPlanars();
  std::vector<Corner> getCorners();
  std::map<int, SemanticFeature> getSemanticFeatures();
  std::vector<ImageFeature> getImageFeatures();

  // Routines to insert features on the global map
  bool insert(const Planar& planar);
  bool insert(const Corner& corner);
  bool insert(const SemanticFeature& landmark, const int& id);
  bool insert(const ImageFeature& image_feature);
  bool directInsert(const Planar& planar);
  bool directInsert(const Corner& corner);

  // Routines to update the location of a given feature
  bool update(const Planar& old_planar, const Planar& new_planar);
  bool update(const Corner& old_corner, const Corner& new_corner);
  bool update(const SemanticFeature& new_landmark, const SemanticFeature& old_landmark, const int& old_landmark_id);
  bool update(const ImageFeature& old_image_feature, const ImageFeature& new_image_feature);

  // Search routines
  bool getAdjacent(const float& x, const float& y, const float& z, const int& layers, vertex_t& node,
                   std::vector<Cell>& adjacent);
  bool getAdjacentFeatures(const float& x, const float& y, const float& z, const int& layers,
                           std::map<int, SemanticFeature>& features);
  bool findNearest(const ImageFeature& input, ImageFeature& nearest, float& ddist);
  bool findNearest(const Corner& input, Corner& nearest, float& sdist);
  bool findNearest(const Planar& input, Planar& nearest, float& sdist);
  bool findNearestOnCell(const ImageFeature& input, ImageFeature& nearest);

  // Downsampling functions for 3D features
  void downsampleCorners();
  void downsamplePlanars();

  // Instanciate a graph
  Graph map_;
  // Vector of vertexes
  std::vector<vertex_t> graph_vertexes_;
  // Vector of vertexes corresponding to the active nodes
  std::vector<vertex_t> active_nodes_vertexes_;
  // Vector of vertexes corresponding to the allocated nodes
  std::vector<vertex_t> allocated_nodes_vertexes_;
  // Flag to tell if the topological map was already initialized
  bool is_initialized_;

private:
  // Align a point by a reference using the rectangle orientation
  void alignPoint(const Point& in_pt, const Point& reference, const float& angle, Point& out_pt);

  // Parameters class
  Parameters params_;
};
}  // namespace vineslam