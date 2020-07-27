#include "../include/localization_node.hpp"

int main(int argc, char** argv)
{
  vineslam::LocalizationNode vineslam_node(argc, argv);
  return 0;
}

namespace vineslam
{

// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

LocalizationNode::LocalizationNode(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "LocalizationNode");
  ros::NodeHandle nh;

  // Set initialize flag default values
  init = true;

  // Load params
  if (!nh.getParam("/localization_node/LocalizationNode/config_path", config_path)) {
    ROS_ERROR("/config_path parameter not found. Shutting down...");
    return;
  }

  // Load config file
  auto config = YAML::LoadFile(config_path);
  // Load camera info parameters
  img_width  = config["camera_info"]["img_width"].as<int>();
  img_height = config["camera_info"]["img_height"].as<int>();
  cam_height = config["camera_info"]["cam_height"].as<float>();
  fx         = config["camera_info"]["fx"].as<float>();
  fy         = config["camera_info"]["fy"].as<float>();
  cx         = config["camera_info"]["cx"].as<float>();
  cy         = config["camera_info"]["cy"].as<float>();
  // Load occupancy grid map parameters
  occ_origin.x   = config["multilayer_mapping"]["grid_map"]["origin"]["x"].as<float>();
  occ_origin.y   = config["multilayer_mapping"]["grid_map"]["origin"]["y"].as<float>();
  occ_resolution = config["multilayer_mapping"]["grid_map"]["resolution"].as<float>();
  occ_width      = config["multilayer_mapping"]["grid_map"]["width"].as<float>();
  occ_height     = config["multilayer_mapping"]["grid_map"]["height"].as<float>();
  use_gps        = config["system"]["use_gps"].as<bool>();
  gps_init_lat   = config["system"]["gps_datum"]["lat"].as<float>();
  gps_init_long  = config["system"]["gps_datum"]["long"].as<float>();

  // Declare the Mappers and Localizer objects
  localizer = new Localizer(config_path);
  grid_map  = new OccupancyMap(config_path);
  mapper2D  = new Mapper2D(config_path);
  mapper3D  = new Mapper3D(config_path);

  // Services
  polar2pose = nh.serviceClient<agrob_map_transform::GetPose>("polar_to_pose");
  set_datum  = nh.serviceClient<agrob_map_transform::SetDatum>("datum");

  // Synchronize subscribers of both topics
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(
      nh, "/left_image", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(
      nh, "/depth_image", 1);
  message_filters::Subscriber<vision_msgs::Detection2DArray> detections_sub(
      nh, "/detections", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image,
                                    sensor_msgs::Image,
                                    vision_msgs::Detection2DArray>
      sync(left_image_sub, depth_image_sub, detections_sub, 10);
  sync.registerCallback(
      boost::bind(&LocalizationNode::callbackFct, this, _1, _2, _3));

  // Odometry subscription
  ros::Subscriber odom_subscriber =
      nh.subscribe("/odom", 1, &LocalizationNode::odomListener, this);
  ros::Subscriber gps_subscriber =
      nh.subscribe("/fix", 1, &LocalizationNode::gpsListener, this);

  // Publish maps and particle filter
  mapOCC_publisher =
      nh.advertise<nav_msgs::OccupancyGrid>("/vineslam/occupancyMap", 1);
  map2D_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map2D", 1);
  map3D_features_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/vineslam/map3D/SURF", 1);
  map3D_corners_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners", 1);
  map3D_planes_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/ground", 1);
  map3D_debug_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/debug", 1);
  normal_pub = nh.advertise<visualization_msgs::Marker>("/map3D/ground_normal", 1);
  pose_publisher  = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  odom_publisher  = nh.advertise<nav_msgs::Odometry>("/vineslam/odom", 1);
  gps_publisher   = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/gps", 1);
  path_publisher  = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);

  // GNSS varibales
  if (use_gps) {
    datum_autocorrection_stage = 0;
    global_counter             = 0;
  }

  // Load map
  MapParser parser(config_path);
  parser.parseFile(*grid_map);

  ros::spin();
  ROS_INFO("ROS shutting down...");
}

LocalizationNode::~LocalizationNode() = default;

// --------------------------------------------------------------------------------
// ----- Callbacks and observation functions
// --------------------------------------------------------------------------------

void LocalizationNode::callbackFct(const sensor_msgs::ImageConstPtr& left_image,
                                   const sensor_msgs::ImageConstPtr& depth_image,
                                   const vision_msgs::Detection2DArrayConstPtr& dets)
{
  // ---------- Publish Multi-layer map ------------- //
  // Publish the grid map
  publishGridMap(depth_image->header);
  // Publish the 2D map
  publish2DMap(depth_image->header);
  // Publish 3D maps
  publish3DMap();
  // ------------------------------------------------ //
}

void LocalizationNode::odomListener(const nav_msgs::OdometryConstPtr& msg) {}

void LocalizationNode::gpsListener(const sensor_msgs::NavSatFixConstPtr& msg) {}

bool LocalizationNode::getGNSSHeading(const pose&             gps_odom,
                                      const std_msgs::Header& header)
{
}

void LocalizationNode::computeObsv(const sensor_msgs::Image& depth_img,
                                   const int&                xmin,
                                   const int&                ymin,
                                   const int&                xmax,
                                   const int&                ymax,
                                   float&                    depth,
                                   float&                    bearing) const
{
}

// --------------------------------------------------------------------------------
// ----- Visualization
// --------------------------------------------------------------------------------

void LocalizationNode::publishGridMap(const std_msgs::Header& header)
{
  // Define ROS occupancy grid map
  nav_msgs::OccupancyGrid occ_map;
  occ_map.header          = header;
  occ_map.header.frame_id = "map";

  // Set the map metadata
  nav_msgs::MapMetaData metadata;
  metadata.origin.position.x    = occ_origin.x;
  metadata.origin.position.y    = occ_origin.y;
  metadata.origin.position.z    = occ_origin.z;
  metadata.origin.orientation.x = 0.;
  metadata.origin.orientation.y = 0.;
  metadata.origin.orientation.z = 0.;
  metadata.origin.orientation.w = 1.;
  metadata.resolution           = occ_resolution;
  metadata.width                = occ_width / occ_resolution;
  metadata.height               = occ_height / occ_resolution;
  occ_map.info                  = metadata;

  // Fill the occupancy grid map
  occ_map.data.resize(metadata.width * metadata.height);
  // - compute x and y bounds
  int xmin = static_cast<int>(occ_origin.x / occ_resolution);
  int xmax = static_cast<int>((float)xmin + occ_width / occ_resolution - 1);
  int ymin = static_cast<int>(occ_origin.y / occ_resolution);
  int ymax = static_cast<int>((float)ymin + occ_height / occ_resolution - 1);
  for (int i = xmin; i < xmax; i++) {
    for (int j = ymin; j < ymax; j++) {
      int8_t number_objs = (*grid_map)(i, j).landmarks.size() +
                           (*grid_map)(i, j).surf_features.size() +
                           (*grid_map)(i, j).corner_features.size();

      int m_i = i - static_cast<int>(occ_origin.x / occ_resolution);
      int m_j = j - static_cast<int>(occ_origin.y / occ_resolution);
      int idx = m_i + m_j * static_cast<int>((occ_width / occ_resolution));

      occ_map.data[idx] = number_objs * 10;
    }
  }

  // Publish the map
  mapOCC_publisher.publish(occ_map);
}

void LocalizationNode::publish2DMap(const std_msgs::Header&   header)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker      marker;
  visualization_msgs::MarkerArray ellipse_array;
  visualization_msgs::Marker      ellipse;

  // Define marker layout
  marker.ns                 = "/markers";
  marker.type               = visualization_msgs::Marker::CYLINDER;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.scale.x            = 0.1;
  marker.scale.y            = 0.1;
  marker.scale.z            = 0.3;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.r            = 0.0f;
  marker.color.g            = 0.0f;
  marker.color.b            = 1.0f;
  marker.color.a            = 1.0;
  marker.lifetime           = ros::Duration();

  // Define marker layout
  ellipse.ns                 = "/ellipses";
  ellipse.type               = visualization_msgs::Marker::CYLINDER;
  ellipse.action             = visualization_msgs::Marker::ADD;
  ellipse.scale.z            = 0.01f;
  ellipse.pose.orientation.x = 0.0f;
  ellipse.pose.orientation.y = 0.0f;
  ellipse.color.r            = 0.0f;
  ellipse.color.g            = 1.0f;
  ellipse.color.b            = 0.0f;
  ellipse.color.a            = 1.0f;
  ellipse.lifetime           = ros::Duration();

  // Publish markers
  int id = 1;
  for (auto& it : (*grid_map)) {
    for (const auto& m_sfeature : it.landmarks) {
      // Draw sfeature mean
      marker.id              = id;
      marker.header          = header;
      marker.header.frame_id = "map";
      marker.pose.position.x = m_sfeature.second.pos.x;
      marker.pose.position.y = m_sfeature.second.pos.y;
      marker.pose.position.z = m_sfeature.second.pos.z;

      marker_array.markers.push_back(marker);

      // Draw sfeature standard deviation
      tf2::Quaternion q;
      q.setRPY(0, 0, m_sfeature.second.gauss.theta);

      ellipse.id                 = id;
      ellipse.header             = header;
      ellipse.header.frame_id    = "map";
      ellipse.pose.position.x    = m_sfeature.second.pos.x;
      ellipse.pose.position.y    = m_sfeature.second.pos.y;
      ellipse.pose.position.z    = m_sfeature.second.pos.z;
      ellipse.scale.x            = 3 * m_sfeature.second.gauss.stdev.x;
      ellipse.scale.y            = 3 * m_sfeature.second.gauss.stdev.y;
      ellipse.pose.orientation.x = q.x();
      ellipse.pose.orientation.y = q.y();
      ellipse.pose.orientation.z = q.z();
      ellipse.pose.orientation.w = q.w();

      ellipse_array.markers.push_back(ellipse);

      id++;
    }
  }

  map2D_publisher.publish(marker_array);
  map2D_publisher.publish(ellipse_array);
}

void LocalizationNode::publish3DMap()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (const auto& it : *grid_map) {
    for (const auto& feature : it.surf_features) {
      pcl::PointXYZRGB m_pt(feature.r, feature.g, feature.b);
      m_pt.x = feature.pos.x;
      m_pt.y = feature.pos.y;
      m_pt.z = feature.pos.z;

      feature_cloud->points.push_back(m_pt);
    }

    for (const auto& corner : it.corner_features) {
      pcl::PointXYZI m_pt(static_cast<float>(corner.which_plane));
      m_pt.x = corner.pos.x;
      m_pt.y = corner.pos.y;
      m_pt.z = corner.pos.z;

      corner_cloud->points.push_back(m_pt);
    }
  }

  feature_cloud->header.frame_id = "map";
  corner_cloud->header.frame_id  = "map";
  map3D_features_publisher.publish(feature_cloud);
  map3D_corners_publisher.publish(corner_cloud);
}

void LocalizationNode::publish3DMap(const Plane& plane, const ros::Publisher& pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointXYZI m_pt(0);

  for (const auto& pt : plane.points) {
    m_pt.x = pt.x;
    m_pt.y = pt.y;
    m_pt.z = pt.z;

    cloud_out->points.push_back(m_pt);
  }

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

void LocalizationNode::publish3DMap(const std::vector<Corner>& corners,
                                    const ros::Publisher&      pub)
{  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointXYZI m_pt(0);

  for (const auto& corner : corners) {
    m_pt.x = corner.pos.x;
    m_pt.y = corner.pos.y;
    m_pt.z = corner.pos.z;

    cloud_out->points.push_back(m_pt);
  }

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

} // namespace vineslam
