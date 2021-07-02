#include <vineslam/mapping/topological_map.hpp>

namespace vineslam
{
// TopologicalMap::TopologicalMap()
//{
//}

void TopologicalMap::polar2Enu(const double& datum_lat, const double& datum_lon, const double& datum_alt,
                               const double& datum_head)
{
  // Declare the geodetic operator and set the datum
  Geodetic l_geodetic_converter(datum_lat, datum_lon, datum_alt);

  // Go through every vertex and compute its enu position on the map
  for (size_t i = 0; i < graph_vertexes_.size(); i++)
  {
    // Compute the enu location of the vertex
    double e, n, u;
    l_geodetic_converter.geodetic2ned(map_[graph_vertexes_[i]].center_.lat_, map_[graph_vertexes_[i]].center_.lon_,
                                      datum_alt, e, n, u);

    // Rotate the obtained point considering the gnss heading
    Point enu(n, -e, 0);
    Point corrected_point = enu;
    corrected_point.x_ = +(enu.x_ * std::cos(+datum_head + M_PI_2) - enu.y_ * std::sin(+datum_head + M_PI_2));
    corrected_point.y_ = -(enu.x_ * std::sin(+datum_head + M_PI_2) + enu.y_ * std::cos(+datum_head + M_PI_2));

    // Save the result
    map_[graph_vertexes_[i]].center_.x_ = corrected_point.x_;
    map_[graph_vertexes_[i]].center_.y_ = corrected_point.y_;

    // Compute the enu location of the rectangle first vertex
    l_geodetic_converter.geodetic2ned(map_[graph_vertexes_[i]].rectangle_[0].lat_,
                                      map_[graph_vertexes_[i]].rectangle_[0].lon_, datum_alt, e, n, u);

    // Rotate the obtained point considering the gnss heading
    enu = Point(n, -e, 0);
    corrected_point.x_ = +(enu.x_ * std::cos(+datum_head + M_PI_2) - enu.y_ * std::sin(+datum_head + M_PI_2));
    corrected_point.y_ = -(enu.x_ * std::sin(+datum_head + M_PI_2) + enu.y_ * std::cos(+datum_head + M_PI_2));

    // Save the result
    map_[graph_vertexes_[i]].rectangle_[0].x_ = corrected_point.x_;
    map_[graph_vertexes_[i]].rectangle_[0].y_ = corrected_point.y_;

    // Compute the enu location of the rectangle first vertex
    l_geodetic_converter.geodetic2ned(map_[graph_vertexes_[i]].rectangle_[1].lat_,
                                      map_[graph_vertexes_[i]].rectangle_[1].lon_, datum_alt, e, n, u);

    // Rotate the obtained point considering the gnss heading
    enu = Point(n, -e, 0);
    corrected_point.x_ = +(enu.x_ * std::cos(+datum_head + M_PI_2) - enu.y_ * std::sin(+datum_head + M_PI_2));
    corrected_point.y_ = -(enu.x_ * std::sin(+datum_head + M_PI_2) + enu.y_ * std::cos(+datum_head + M_PI_2));

    // Save the result
    map_[graph_vertexes_[i]].rectangle_[1].x_ = corrected_point.x_;
    map_[graph_vertexes_[i]].rectangle_[1].y_ = corrected_point.y_;
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

}  // namespace vineslam
