#include <vineslam/mapping/topological_map.hpp>

namespace vineslam
{
TopologicalMap::TopologicalMap() : is_initialized_(false)
{
}

void TopologicalMap::init(OccupancyMap* input_map, const double& datum_head)
{
  // Store the input occupancy map into the graph-like structure
  // ...

  // Set the initialization flag
  is_initialized_ = true;
}

void TopologicalMap::polar2Enu(const double& datum_lat, const double& datum_lon, const double& datum_alt,
                               const double& datum_head)
{
  // Declare the geodetic operator and set the datum
  Geodetic l_geodetic_converter(datum_lat, datum_lon, datum_alt);

  // Go through every vertex and compute its enu position on the map
  for (size_t i = 0; i < graph_vertexes_.size(); i++)
  {
    double e, n, u;
    Point corrected_point;

    // Resize rectangle array
    map_[graph_vertexes_[i]].rectangle_.resize(4);

    // Compute the enu location of the rectangle first vertex
    l_geodetic_converter.geodetic2ned(map_[graph_vertexes_[i]].rectangle_[0].lat_,
                                      map_[graph_vertexes_[i]].rectangle_[0].lon_, datum_alt, e, n, u);

    // Rotate the obtained point considering the gnss heading
    Point enu_c1(n, -e, 0);
    corrected_point.x_ = +(enu_c1.x_ * std::cos(datum_head + M_PI_2) - enu_c1.y_ * std::sin(datum_head + M_PI_2));
    corrected_point.y_ = -(enu_c1.x_ * std::sin(datum_head + M_PI_2) + enu_c1.y_ * std::cos(datum_head + M_PI_2));

    // Save the result
    map_[graph_vertexes_[i]].rectangle_[0].x_ = corrected_point.x_;
    map_[graph_vertexes_[i]].rectangle_[0].y_ = corrected_point.y_;

    // Compute the enu location of the rectangle first vertex
    l_geodetic_converter.geodetic2ned(map_[graph_vertexes_[i]].rectangle_[1].lat_,
                                      map_[graph_vertexes_[i]].rectangle_[1].lon_, datum_alt, e, n, u);

    // Rotate the obtained point considering the gnss heading
    Point enu_c2(n, -e, 0);
    corrected_point.x_ = +(enu_c2.x_ * std::cos(datum_head + M_PI_2) - enu_c2.y_ * std::sin(datum_head + M_PI_2));
    corrected_point.y_ = -(enu_c2.x_ * std::sin(datum_head + M_PI_2) + enu_c2.y_ * std::cos(datum_head + M_PI_2));

    // Save the result
    map_[graph_vertexes_[i]].rectangle_[1].x_ = corrected_point.x_;
    map_[graph_vertexes_[i]].rectangle_[1].y_ = corrected_point.y_;

    // Compute the enu location of the rectangle first vertex
    l_geodetic_converter.geodetic2ned(map_[graph_vertexes_[i]].rectangle_[2].lat_,
                                      map_[graph_vertexes_[i]].rectangle_[2].lon_, datum_alt, e, n, u);

    // Rotate the obtained point considering the gnss heading
    Point enu_c3(n, -e, 0);
    corrected_point.x_ = +(enu_c3.x_ * std::cos(datum_head + M_PI_2) - enu_c3.y_ * std::sin(datum_head + M_PI_2));
    corrected_point.y_ = -(enu_c3.x_ * std::sin(datum_head + M_PI_2) + enu_c3.y_ * std::cos(datum_head + M_PI_2));

    // Save the result
    map_[graph_vertexes_[i]].rectangle_[2].x_ = corrected_point.x_;
    map_[graph_vertexes_[i]].rectangle_[2].y_ = corrected_point.y_;

    // Compute the enu location of the rectangle first vertex
    l_geodetic_converter.geodetic2ned(map_[graph_vertexes_[i]].rectangle_[3].lat_,
                                      map_[graph_vertexes_[i]].rectangle_[3].lon_, datum_alt, e, n, u);

    // Rotate the obtained point considering the gnss heading
    Point enu_c4(n, -e, 0);
    corrected_point.x_ = +(enu_c4.x_ * std::cos(datum_head + M_PI_2) - enu_c4.y_ * std::sin(datum_head + M_PI_2));
    corrected_point.y_ = -(enu_c4.x_ * std::sin(datum_head + M_PI_2) + enu_c4.y_ * std::cos(datum_head + M_PI_2));

    // Save the result
    map_[graph_vertexes_[i]].rectangle_[3].x_ = corrected_point.x_;
    map_[graph_vertexes_[i]].rectangle_[3].y_ = corrected_point.y_;

    // Set the location of the vertex center
    map_[graph_vertexes_[i]].center_.x_ = (map_[graph_vertexes_[i]].rectangle_[0].x_ + map_[graph_vertexes_[i]].rectangle_[2].x_) / 2.;
    map_[graph_vertexes_[i]].center_.y_ = (map_[graph_vertexes_[i]].rectangle_[0].y_ + map_[graph_vertexes_[i]].rectangle_[2].y_) / 2.;
  }
}

void TopologicalMap::getAdjacentList(vertex_t v, std::vector<uint32_t>* adj_index)
{
  adj_index->clear();
  uint32_t index;
  AdjacencyIterator ai, a_end;
  boost::tie(ai, a_end) = boost::adjacent_vertices(v, map_);

  for (; ai != a_end; ai++)
  {
    index = map_[*ai].index_;
    adj_index->push_back(index);
  }
}

void TopologicalMap::getActiveNodes(const Pose& robot_pose)
{
  // Reset the active nodes
  active_nodes_vertexes_.clear();

  // Go through every vertex
  for (size_t i = 0; i < graph_vertexes_.size(); i++)
  {
    // Compute distance from the robot to the center of the vertex
    Point center = Point(map_[graph_vertexes_[i]].center_.x_, map_[graph_vertexes_[i]].center_.y_, 0.);
    float dist = center.distanceXY(robot_pose.getXYZ());

    // Check if this is an active node
    if (dist <= 20.0)
    {
      active_nodes_vertexes_.push_back(graph_vertexes_[i]);
    }
  }
}

void TopologicalMap::deallocateNodes(const Pose& robot_pose)
{
}

bool TopologicalMap::getNode(const Point& point, vertex_t& node)
{
  // Init vars
  float min_dist = std::numeric_limits<float>::max();
  bool found_solution = false;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    // Compute distance from the point to the center of the vertex
    Point center = Point(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0.);
    float dist = center.distanceXY(point);

    // Find the closest node
    if (dist < min_dist)
    {
      // Check if the point lies inside the node rectangle
      float a = std::fabs(point.x_ - map_[active_nodes_vertexes_[i]].rectangle_[0].x_);
      float b = std::fabs(point.x_ - map_[active_nodes_vertexes_[i]].rectangle_[2].x_);
      float c = std::fabs(point.y_ - map_[active_nodes_vertexes_[i]].rectangle_[0].y_);
      float d = std::fabs(point.y_ - map_[active_nodes_vertexes_[i]].rectangle_[2].y_);

      float calculated_area = (a + b) * (c + d);
      float rectangle_area = (std::fabs(map_[active_nodes_vertexes_[i]].rectangle_[0].x_ -
                                        map_[active_nodes_vertexes_[i]].rectangle_[2].x_) *
                              std::fabs(map_[active_nodes_vertexes_[i]].rectangle_[0].y_ -
                                        map_[active_nodes_vertexes_[i]].rectangle_[2].y_));

      // We found a possible node for the point
      if (calculated_area == rectangle_area)
      {
        node = active_nodes_vertexes_[i];
        found_solution = true;
      }
    }
  }

  return found_solution;
}

void TopologicalMap::getCell(const Point& point, Cell* cell)
{
}

}  // namespace vineslam
