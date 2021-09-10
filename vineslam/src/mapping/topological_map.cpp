#include <vineslam/mapping/topological_map.hpp>

namespace vineslam
{
TopologicalMap::TopologicalMap(const Parameters& params)
{
  is_initialized_ = false;
  params_ = params;
}

void TopologicalMap::init(OccupancyMap* input_map, const double& heading)
{
  // Store the input occupancy map into the graph-like structure
  // ...

  // Release the temporary occupancy grid map memory
  // ...

  // Mapping between rotated and non-rotated rectangles to allocate grid map memory
  for (size_t i = 0; i < graph_vertexes_.size(); i++)
  {
    map_[graph_vertexes_[i]].rectangle_orientation_ += (heading + M_PI_2);
    Point center(map_[graph_vertexes_[i]].center_.x_, map_[graph_vertexes_[i]].center_.y_, 0);

    Point p1(map_[graph_vertexes_[i]].rectangle_[0].x_, map_[graph_vertexes_[i]].rectangle_[0].y_, 0);
    Point p3(map_[graph_vertexes_[i]].rectangle_[2].x_, map_[graph_vertexes_[i]].rectangle_[2].y_, 0);
    Point out_p1, out_p3;

    alignPoint(p1, center, map_[graph_vertexes_[i]].rectangle_orientation_, out_p1);
    alignPoint(p3, center, map_[graph_vertexes_[i]].rectangle_orientation_, out_p3);

    map_[graph_vertexes_[i]].aligned_rectangle_.resize(4);
    map_[graph_vertexes_[i]].aligned_rectangle_[0].x_ = out_p1.x_;
    map_[graph_vertexes_[i]].aligned_rectangle_[0].y_ = out_p1.y_;
    map_[graph_vertexes_[i]].aligned_rectangle_[1].x_ = out_p1.x_;
    map_[graph_vertexes_[i]].aligned_rectangle_[1].y_ = out_p3.y_;
    map_[graph_vertexes_[i]].aligned_rectangle_[2].x_ = out_p3.x_;
    map_[graph_vertexes_[i]].aligned_rectangle_[2].y_ = out_p3.y_;
    map_[graph_vertexes_[i]].aligned_rectangle_[3].x_ = out_p3.x_;
    map_[graph_vertexes_[i]].aligned_rectangle_[3].y_ = out_p1.y_;

    // Set the class flags
    map_[graph_vertexes_[i]].has_file_ = false;
  }

  // Set the class flags
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
    map_[graph_vertexes_[i]].center_.x_ =
        (map_[graph_vertexes_[i]].rectangle_[0].x_ + map_[graph_vertexes_[i]].rectangle_[2].x_) / 2.;
    map_[graph_vertexes_[i]].center_.y_ =
        (map_[graph_vertexes_[i]].rectangle_[0].y_ + map_[graph_vertexes_[i]].rectangle_[2].y_) / 2.;
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
  // Go through every allocated vertex
  for (size_t i = 0; i < allocated_nodes_vertexes_.size(); i++)
  {
    // Compute distance from the robot to the center of the vertex
    Point center =
        Point(map_[allocated_nodes_vertexes_[i]].center_.x_, map_[allocated_nodes_vertexes_[i]].center_.y_, 0.);
    float dist = center.distanceXY(robot_pose.getXYZ());

    // Check if we want to deallocate this node
    if (dist > 20.0)
    {
      // Remove the node from the allocated nodes array
      allocated_nodes_vertexes_.erase(allocated_nodes_vertexes_.begin() + i);

      // Write the map to the corresponding xml file
      // Create local parameter structure to feed the occupancy grid map
      Parameters l_params = params_;
      l_params.gridmap_width_ = map_[allocated_nodes_vertexes_[i]].grid_map_->width_;
      l_params.gridmap_lenght_ = map_[allocated_nodes_vertexes_[i]].grid_map_->lenght_;
      l_params.gridmap_origin_x_ = map_[allocated_nodes_vertexes_[i]].grid_map_->origin_.x_;
      l_params.gridmap_origin_y_ = map_[allocated_nodes_vertexes_[i]].grid_map_->origin_.y_;
      MapWriter mw(l_params, map_[allocated_nodes_vertexes_[i]].index_);
      mw.writeToFile(map_[allocated_nodes_vertexes_[i]].grid_map_, l_params);

      // Set the has_file flag
      map_[allocated_nodes_vertexes_[i]].has_file_ = true;

      // Free the map memory
      free(map_[allocated_nodes_vertexes_[i]].grid_map_);
    }
  }
}

void TopologicalMap::allocateNodeMap(const vertex_t& node)
{
  // Create local parameter structure to feed the occupancy grid map
  Parameters l_params = params_;
  l_params.gridmap_width_ = std::fabs(map_[node].aligned_rectangle_[0].x_ - map_[node].aligned_rectangle_[2].x_);
  l_params.gridmap_lenght_ = std::fabs(map_[node].aligned_rectangle_[0].y_ - map_[node].aligned_rectangle_[2].y_);
  l_params.gridmap_origin_x_ = map_[node].center_.x_ - l_params.gridmap_width_ / 2;
  l_params.gridmap_origin_y_ = map_[node].center_.y_ - l_params.gridmap_lenght_ / 2;

  // Allocate memory for the map
  map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
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
      if (map_[active_nodes_vertexes_[i]].aligned_rectangle_.size() != 4)
      {
        continue;
      }

      // Check if the point lies inside the node rectangle
      // - transform the point to the aligned rectangles space
      Point aligned_point;
      alignPoint(point, center, map_[active_nodes_vertexes_[i]].rectangle_orientation_, aligned_point);
      // - calculate the distance from the aligned point to the rectangle center component-wise
      float dist_x = std::fabs(aligned_point.x_ - center.x_);
      float dist_y = std::fabs(aligned_point.y_ - center.y_);
      float size_x = std::fabs(map_[active_nodes_vertexes_[i]].aligned_rectangle_[0].x_ -
                               map_[active_nodes_vertexes_[i]].aligned_rectangle_[2].x_);
      float size_y = std::fabs(map_[active_nodes_vertexes_[i]].aligned_rectangle_[0].y_ -
                               map_[active_nodes_vertexes_[i]].aligned_rectangle_[2].y_);

      // We found a possible node for the point
      if (dist_x < size_x / 2. && dist_y < size_y / 2.)
      {
        min_dist = dist;
        node = active_nodes_vertexes_[i];
        found_solution = true;
      }
    }
  }

  return found_solution;
}

bool TopologicalMap::getCell(Point& point, Cell& cell, vertex_t& node, bool read_only)
{
  // Get the node corresponding to the input point
  bool get_node = getNode(point, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    // if this call is read only, we have nothing to do
    // else, we will allocate the map
    if (read_only)
    {
      return false;
    }
    else
    {
      if (map_[node].has_file_)  // Map is not allocated but is saved on a file
      {
        // Load file to an occupancy grid map structure
        Parameters l_params;
        MapParser map_parser(l_params);
        if (!map_parser.parseHeader(&l_params))
        {
          std::cout << "Map input file not found." << std::endl;
          return false;
        }
        else
        {
          map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
        }

        if (!map_parser.parseFile(&(*map_[node].grid_map_)))
        {
          std::cout << "Map input file not found." << std::endl;
          return false;
        }
      }
      else  // Map is not allocated neither is saved on a file
      {
        // create the occupancy grid map structure
        allocateNodeMap(node);
      }
    }
  }

  // If we got here, the occupancy grid map was allocated. So, we can access the cell
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(point, center, map_[node].rectangle_orientation_, aligned_point);
  point.x_ = aligned_point.x_;
  point.y_ = aligned_point.y_;

  cell = (*map_[node].grid_map_)(point.x_, point.y_, point.z_);

  return true;
}

void TopologicalMap::alignPoint(const Point& in_pt, const Point& reference, const float& angle, Point& out_pt)
{
  float x1 = in_pt.x_ - reference.x_;
  float y1 = in_pt.y_ - reference.y_;

  out_pt.x_ = x1 * std::cos(angle) - y1 * std::sin(angle);
  out_pt.y_ = x1 * std::sin(angle) + y1 * std::cos(angle);

  out_pt.x_ += reference.x_;
  out_pt.y_ += reference.y_;
}

std::vector<Planar> TopologicalMap::getPlanars(const float& x, const float& y, const float& z)
{
  Cell c;
  vertex_t node;
  Point pt(x, y, z);
  std::vector<Planar> planars;

  // Get the cell
  if (!getCell(pt, c, node))
  {
    return planars;
  }

  // Get rotation parameters
  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = -map_[node].rectangle_orientation_;

  // Get planar features on the cell
  if (c.data != nullptr)
  {
    if (c.data->planar_features_ != nullptr)
    {
      std::vector<Planar> l_planars = *c.data->planar_features_;
      for (auto& ft : l_planars)
      {
        Point pt;
        alignPoint(ft.pos_, center, angle, pt);
        ft.pos_.x_ = pt.x_;
        ft.pos_.y_ = pt.y_;

        planars.push_back(ft);
      }
    }
  }

  return planars;
}

std::vector<Corner> TopologicalMap::getCorners(const float& x, const float& y, const float& z)
{
  Cell c;
  vertex_t node;
  Point pt(x, y, z);
  std::vector<Corner> corners;

  // Get the cell
  if (!getCell(pt, c, node))
  {
    return corners;
  }

  // Get rotation parameters
  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = -map_[node].rectangle_orientation_;

  // Get corner features on the cell
  if (c.data != nullptr)
  {
    if (c.data->corner_features_ != nullptr)
    {
      std::vector<Corner> l_corners = *c.data->corner_features_;
      for (auto& ft : l_corners)
      {
        Point pt;
        alignPoint(ft.pos_, center, angle, pt);
        ft.pos_.x_ = pt.x_;
        ft.pos_.y_ = pt.y_;

        corners.push_back(ft);
      }
    }
  }

  return corners;
}

std::map<int, SemanticFeature> TopologicalMap::getSemanticFeatures(const float& x, const float& y)
{
  Cell c;
  vertex_t node;
  Point pt(x, y, 0);
  std::map<int, SemanticFeature> landmarks;

  // Get the cell
  if (!getCell(pt, c, node))
  {
    return landmarks;
  }

  // Get rotation parameters
  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = -map_[node].rectangle_orientation_;

  // Get semantic features on the cell
  if (c.data != nullptr)
  {
    if (c.data->landmarks_ != nullptr)
    {
      std::map<int, SemanticFeature> l_landmarks = *c.data->landmarks_;
      for (auto& ft : l_landmarks)
      {
        Point pt;
        alignPoint(ft.second.pos_, center, angle, pt);
        ft.second.pos_.x_ = pt.x_;
        ft.second.pos_.y_ = pt.y_;

        landmarks[ft.first] = ft.second;
      }
    }
  }

  return landmarks;
}

std::vector<ImageFeature> TopologicalMap::getImageFeatures(const float& x, const float& y, const float& z)
{
  Cell c;
  vertex_t node;
  Point pt(x, y, z);
  std::vector<ImageFeature> image_features;

  // Get the cell
  if (!getCell(pt, c, node))
  {
    return image_features;
  }

  // Get rotation parameters
  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = -map_[node].rectangle_orientation_;

  // Get image features on the cell
  if (c.data != nullptr)
  {
    if (c.data->surf_features_ != nullptr)
    {
      std::vector<ImageFeature> l_image_features = *c.data->surf_features_;
      for (auto& ft : l_image_features)
      {
        Point pt;
        alignPoint(ft.pos_, center, angle, pt);
        ft.pos_.x_ = pt.x_;
        ft.pos_.y_ = pt.y_;

        image_features.push_back(ft);
      }
    }
  }

  return image_features;
}

std::vector<Planar> TopologicalMap::getPlanars()
{
  std::vector<Planar> planars;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    Point center(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0);
    float angle = -map_[active_nodes_vertexes_[i]].rectangle_orientation_;

    if (map_[active_nodes_vertexes_[i]].grid_map_ != nullptr)
    {
      std::vector<Planar> l_planars = map_[active_nodes_vertexes_[i]].grid_map_->getPlanars();
      for (auto& ft : l_planars)
      {
        Point pt;
        alignPoint(ft.pos_, center, angle, pt);
        ft.pos_.x_ = pt.x_;
        ft.pos_.y_ = pt.y_;

        planars.push_back(ft);
      }
    }
  }

  return planars;
}

std::vector<Corner> TopologicalMap::getCorners()
{
  std::vector<Corner> corners;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    Point center(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0);
    float angle = -map_[active_nodes_vertexes_[i]].rectangle_orientation_;

    if (map_[active_nodes_vertexes_[i]].grid_map_ != nullptr)
    {
      std::vector<Corner> l_corners = map_[active_nodes_vertexes_[i]].grid_map_->getCorners();
      for (auto& ft : l_corners)
      {
        Point pt;
        alignPoint(ft.pos_, center, angle, pt);
        ft.pos_.x_ = pt.x_;
        ft.pos_.y_ = pt.y_;

        corners.push_back(ft);
      }
    }
  }

  return corners;
}

std::map<int, SemanticFeature> TopologicalMap::getSemanticFeatures()
{
  std::map<int, SemanticFeature> landmarks;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    Point center(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0);
    float angle = -map_[active_nodes_vertexes_[i]].rectangle_orientation_;

    if (map_[active_nodes_vertexes_[i]].grid_map_ != nullptr)
    {
      std::map<int, SemanticFeature> l_landmarks = map_[active_nodes_vertexes_[i]].grid_map_->getLandmarks();
      for (auto& ft : l_landmarks)
      {
        Point pt;
        alignPoint(ft.second.pos_, center, angle, pt);
        ft.second.pos_.x_ = pt.x_;
        ft.second.pos_.y_ = pt.y_;

        landmarks[ft.first] = ft.second;
      }
    }
  }

  return landmarks;
}

std::vector<ImageFeature> TopologicalMap::getImageFeatures()
{
  std::vector<ImageFeature> images_features;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    Point center(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0);
    float angle = -map_[active_nodes_vertexes_[i]].rectangle_orientation_;

    if (map_[active_nodes_vertexes_[i]].grid_map_ != nullptr)
    {
      std::vector<ImageFeature> l_image_features = map_[active_nodes_vertexes_[i]].grid_map_->getImageFeatures();
      for (auto& ft : l_image_features)
      {
        Point pt;
        alignPoint(ft.pos_, center, angle, pt);
        ft.pos_.x_ = pt.x_;
        ft.pos_.y_ = pt.y_;

        images_features.push_back(ft);
      }
    }
  }

  return images_features;
}

bool TopologicalMap::insert(const Planar& planar)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(planar.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      Parameters l_params;
      MapParser map_parser(l_params);
      if (!map_parser.parseHeader(&l_params))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }
      else
      {
        map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
      }

      if (!map_parser.parseFile(&(*map_[node].grid_map_)))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(planar.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  Planar transformed_ft = planar;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  map_[node].grid_map_->insert(transformed_ft);

  return true;
}

bool TopologicalMap::insert(const Corner& corner)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(corner.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      Parameters l_params;
      MapParser map_parser(l_params);
      if (!map_parser.parseHeader(&l_params))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }
      else
      {
        map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
      }

      if (!map_parser.parseFile(&(*map_[node].grid_map_)))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(corner.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  Corner transformed_ft = corner;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  map_[node].grid_map_->insert(transformed_ft);

  return true;
}

bool TopologicalMap::insert(const SemanticFeature& landmark, const int& id)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(landmark.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      Parameters l_params;
      MapParser map_parser(l_params);
      if (!map_parser.parseHeader(&l_params))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }
      else
      {
        map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
      }

      if (!map_parser.parseFile(&(*map_[node].grid_map_)))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(landmark.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  SemanticFeature transformed_ft = landmark;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  map_[node].grid_map_->insert(transformed_ft, id);

  return true;
}

bool TopologicalMap::insert(const ImageFeature& image_feature)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(image_feature.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      Parameters l_params;
      MapParser map_parser(l_params);
      if (!map_parser.parseHeader(&l_params))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }
      else
      {
        map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
      }

      if (!map_parser.parseFile(&(*map_[node].grid_map_)))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(image_feature.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  ImageFeature transformed_ft = image_feature;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  map_[node].grid_map_->insert(transformed_ft);

  return true;
}

bool TopologicalMap::directInsert(const Planar& planar)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(planar.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      Parameters l_params;
      MapParser map_parser(l_params);
      if (!map_parser.parseHeader(&l_params))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }
      else
      {
        map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
      }

      if (!map_parser.parseFile(&(*map_[node].grid_map_)))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(planar.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  Planar transformed_ft = planar;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  map_[node].grid_map_->directInsert(transformed_ft);

  return true;
}

bool TopologicalMap::directInsert(const Corner& corner)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(corner.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      Parameters l_params;
      MapParser map_parser(l_params);
      if (!map_parser.parseHeader(&l_params))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }
      else
      {
        map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
      }

      if (!map_parser.parseFile(&(*map_[node].grid_map_)))
      {
        std::cout << "Map input file not found." << std::endl;
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(corner.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  Corner transformed_ft = corner;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  map_[node].grid_map_->directInsert(transformed_ft);

  return true;
}

bool TopologicalMap::update(const Planar& old_planar, const Planar& new_planar)
{
  // Get the nodes corresponding to the input features
  vertex_t old_node, new_node;
  bool get_old_node = getNode(old_planar.pos_, old_node);
  bool get_new_node = getNode(new_planar.pos_, new_node);

  // Check if the nodes are active
  if (!get_old_node || !get_new_node || (map_[old_node].index_ != map_[new_node].index_))
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[new_node].grid_map_ == nullptr)
  {
    return false;
  }
  else
  {
    // Transform the features
    Point aligned_old_point, aligned_new_point, center(map_[new_node].center_.x_, map_[new_node].center_.y_, 0);
    alignPoint(old_planar.pos_, center, map_[new_node].rectangle_orientation_, aligned_old_point);
    alignPoint(new_planar.pos_, center, map_[new_node].rectangle_orientation_, aligned_new_point);
    Planar transformed_old_ft = old_planar;
    Planar transformed_new_ft = new_planar;
    transformed_old_ft.pos_.x_ = aligned_old_point.x_;
    transformed_old_ft.pos_.y_ = aligned_old_point.y_;
    transformed_new_ft.pos_.x_ = aligned_new_point.x_;
    transformed_new_ft.pos_.y_ = aligned_new_point.y_;

    // Update the features
    map_[new_node].grid_map_->update(transformed_old_ft, transformed_new_ft);
  }

  return true;
}

bool TopologicalMap::update(const Corner& old_corner, const Corner& new_corner)
{
  // Get the nodes corresponding to the input features
  vertex_t old_node, new_node;
  bool get_old_node = getNode(old_corner.pos_, old_node);
  bool get_new_node = getNode(new_corner.pos_, new_node);

  // Check if the nodes are active
  if (!get_old_node || !get_new_node || (map_[old_node].index_ != map_[new_node].index_))
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[new_node].grid_map_ == nullptr)
  {
    return false;
  }
  else
  {
    // Transform the features
    Point aligned_old_point, aligned_new_point, center(map_[new_node].center_.x_, map_[new_node].center_.y_, 0);
    alignPoint(old_corner.pos_, center, map_[new_node].rectangle_orientation_, aligned_old_point);
    alignPoint(new_corner.pos_, center, map_[new_node].rectangle_orientation_, aligned_new_point);
    Corner transformed_old_ft = old_corner;
    Corner transformed_new_ft = new_corner;
    transformed_old_ft.pos_.x_ = aligned_old_point.x_;
    transformed_old_ft.pos_.y_ = aligned_old_point.y_;
    transformed_new_ft.pos_.x_ = aligned_new_point.x_;
    transformed_new_ft.pos_.y_ = aligned_new_point.y_;

    // Update the features
    map_[new_node].grid_map_->update(transformed_old_ft, transformed_new_ft);
  }

  return true;
}

bool TopologicalMap::update(const SemanticFeature& new_landmark, const SemanticFeature& old_landmark,
                            const int& old_landmark_id)
{
  // Get the nodes corresponding to the input features
  vertex_t old_node, new_node;
  bool get_old_node = getNode(old_landmark.pos_, old_node);
  bool get_new_node = getNode(new_landmark.pos_, new_node);

  // Check if the nodes are active
  if (!get_old_node || !get_new_node || (map_[old_node].index_ != map_[new_node].index_))
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[new_node].grid_map_ == nullptr)
  {
    return false;
  }
  else
  {
    // Transform the features
    Point aligned_old_point, aligned_new_point, center(map_[new_node].center_.x_, map_[new_node].center_.y_, 0);
    alignPoint(old_landmark.pos_, center, map_[new_node].rectangle_orientation_, aligned_old_point);
    alignPoint(new_landmark.pos_, center, map_[new_node].rectangle_orientation_, aligned_new_point);
    SemanticFeature transformed_old_ft = old_landmark;
    SemanticFeature transformed_new_ft = old_landmark;
    transformed_old_ft.pos_.x_ = aligned_old_point.x_;
    transformed_old_ft.pos_.y_ = aligned_old_point.y_;
    transformed_new_ft.pos_.x_ = aligned_new_point.x_;
    transformed_new_ft.pos_.y_ = aligned_new_point.y_;

    // Update the features
    map_[new_node].grid_map_->update(transformed_new_ft, transformed_old_ft, old_landmark_id);
  }

  return true;
}

bool TopologicalMap::update(const ImageFeature& old_image_feature, const ImageFeature& new_image_feature)
{
  // Get the nodes corresponding to the input features
  vertex_t old_node, new_node;
  bool get_old_node = getNode(old_image_feature.pos_, old_node);
  bool get_new_node = getNode(new_image_feature.pos_, new_node);

  // Check if the nodes are active
  if (!get_old_node || !get_new_node || (map_[old_node].index_ != map_[new_node].index_))
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[new_node].grid_map_ == nullptr)
  {
    return false;
  }
  else
  {
    // Transform the features
    Point aligned_old_point, aligned_new_point, center(map_[new_node].center_.x_, map_[new_node].center_.y_, 0);
    alignPoint(old_image_feature.pos_, center, map_[new_node].rectangle_orientation_, aligned_old_point);
    alignPoint(new_image_feature.pos_, center, map_[new_node].rectangle_orientation_, aligned_new_point);
    ImageFeature transformed_old_ft = old_image_feature;
    ImageFeature transformed_new_ft = new_image_feature;
    transformed_old_ft.pos_.x_ = aligned_old_point.x_;
    transformed_old_ft.pos_.y_ = aligned_old_point.y_;
    transformed_new_ft.pos_.x_ = aligned_new_point.x_;
    transformed_new_ft.pos_.y_ = aligned_new_point.y_;

    // Update the features
    map_[new_node].grid_map_->update(transformed_old_ft, transformed_new_ft);
  }

  return true;
}

void TopologicalMap::downsamplePlanars()
{
  // Go through every allocated vertex
  for (size_t i = 0; i < allocated_nodes_vertexes_.size(); i++)
  {
    map_[allocated_nodes_vertexes_[i]].grid_map_->downsamplePlanars();
  }
}

void TopologicalMap::downsampleCorners()
{
  // Go through every allocated vertex
  for (size_t i = 0; i < allocated_nodes_vertexes_.size(); i++)
  {
    map_[allocated_nodes_vertexes_[i]].grid_map_->downsampleCorners();
  }
}
}  // namespace vineslam