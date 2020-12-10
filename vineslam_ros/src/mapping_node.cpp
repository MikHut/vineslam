#include "../include/mapping_node.hpp"

int main(int argc, char** argv)
{
  vineslam::MappingNode mapping_node(argc, argv);
  return 0;
}

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

MappingNode::MappingNode(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "MappingNode");
  ros::NodeHandle nh;

  // Load params
  loadParameters(nh, "/mapping_node", params_);

  // Declare the Mappers and Localizer objects
  lid_mapper_ = new LidarMapper(params_);

  // Scan subscription
  ros::Subscriber scan_subscriber =
      nh.subscribe(params_.pcl_topic_, 1, &VineSLAM_ros::scanListener, dynamic_cast<VineSLAM_ros*>(this));
  // Odometry subscription
  ros::Subscriber odom_subscriber =
      nh.subscribe(params_.odom_topic_, 1, &VineSLAM_ros::odomListener, dynamic_cast<VineSLAM_ros*>(this));

  // Publish maps and particle filter
  map3D_corners_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners", 1);
  map3D_planars_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars", 1);
  planes_local_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map3D/planes_local", 1);
  corners_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners_local", 1);
  planars_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars_local", 1);

  // ROS services
  ros::ServiceServer save_map_srv =
      nh.advertiseService("save_map", &VineSLAM_ros::saveMap, dynamic_cast<VineSLAM_ros*>(this));

  // Get static sensor tfs
  tfScalar roll, pitch, yaw;
  tf::Transform vel2base;
  vel2base.setRotation(
      tf::Quaternion(params_.vel2base_[3], params_.vel2base_[4], params_.vel2base_[5], params_.vel2base_[6]));
  vel2base.setOrigin(tf::Vector3(params_.vel2base_[0], params_.vel2base_[1], params_.vel2base_[2]));
  vel2base = vel2base.inverse();
  tf::Vector3 t = vel2base.getOrigin();
  vel2base.getBasis().getRPY(roll, pitch, yaw);
  lid_mapper_->setVel2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);

  // Set initialization flags default values
  init_flag_ = true;
  init_odom_ = true;

  // Call execution thread
  std::thread th(&MappingNode::loop, this);
  th.detach();

  // ROS spin ...
  ROS_INFO("Done! Execution started.");
  ros::spin();
  ROS_INFO("ROS shutting down...");
}

MappingNode::~MappingNode() = default;

void MappingNode::loop()
{
  // Reset information flags
  input_data_.received_scans_ = false;
  input_data_.received_odometry_ = false;

  while (ros::ok())
  {
    loopOnce();
  }
}

void MappingNode::loopOnce()
{
  // Check if we have all the necessary data
  bool can_continue = input_data_.received_scans_ && input_data_.received_odometry_;

  if (!can_continue)
    return;

  // VineSLAM main loop
  if (init_flag_)
  {
    init();
    init_flag_ = false;
  }
  else
    process();

  // Reset information flags
  input_data_.received_scans_ = false;
  input_data_.received_odometry_ = false;
}

void MappingNode::init()
{
  // ---------------------------------------------------------
  // ----- Initialize Occupancy Grid map
  // ---------------------------------------------------------
  grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0));

  // ---------------------------------------------------------
  // ----- Initialize the multi-layer maps
  // ---------------------------------------------------------
  std::vector<Corner> m_corners;
  std::vector<Planar> m_planars;
  std::vector<SemiPlane> m_planes;
  Plane m_ground_plane;
  lid_mapper_->localMap(input_data_.scan_pts_, m_corners, m_planars, m_planes, m_ground_plane);

  // - Register 3D maps
  lid_mapper_->registerMaps(input_data_.wheel_odom_pose_, m_corners, m_planars, m_planes, *grid_map_);
  grid_map_->downsamplePlanars();

  ROS_INFO("Mapping with known poses has started.");
}

void MappingNode::process()
{
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------
  // ---- Mapping procedures
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------

  // ---------------------------------------------------------
  // ----- Build local maps to use in the localization
  // ---------------------------------------------------------
  std::vector<Corner> m_corners;
  std::vector<Planar> m_planars;
  std::vector<SemiPlane> m_planes;
  Plane m_ground_plane;
  lid_mapper_->localMap(input_data_.scan_pts_, m_corners, m_planars, m_planes, m_ground_plane);

  // ---------------------------------------------------------
  // ----- Register multi-layer map (if performing SLAM)
  // ---------------------------------------------------------
  lid_mapper_->registerMaps(input_data_.wheel_odom_pose_, m_corners, m_planars, m_planes, *grid_map_);
  grid_map_->downsamplePlanars();

  // ---------------------------------------------------------
  // ----- ROS publishers and tf broadcasting
  // ---------------------------------------------------------
  static tf::TransformBroadcaster br;
  tf::Quaternion o2m_q;
  o2m_q.setRPY(init_odom_pose_.R_, init_odom_pose_.P_, init_odom_pose_.Y_);
  tf::Transform odom2map(o2m_q, tf::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_));
  br.sendTransform(tf::StampedTransform(odom2map, ros::Time::now(), "odom", "map"));

  // Publish 3D maps
  //  publish3DMap();
  publish3DMap(m_corners, corners_local_publisher_);
  publish3DMap(m_planars, planars_local_publisher_);
  std::vector<Plane> planes = { m_ground_plane };
  for (const auto& plane : m_planes)
    planes.push_back(plane);
}

}  // namespace vineslam