#include "../include/wildSLAM_ros.hpp"

void wildSLAM_ros::SLAMNode::odomListener(const nav_msgs::OdometryConstPtr& msg)
{
	// Convert odometry msg to pose msg
	tf::Pose            pose;
	geometry_msgs::Pose odom_pose = (*msg).pose.pose;
	tf::poseMsgToTF(odom_pose, pose);

	// Check if yaw is NaN
	double yaw = tf::getYaw(pose.getRotation());
	if (yaw != yaw)
		yaw = 0;

	// If it is the first iteration - initialize the Pose
	// relative to the previous frame
	if (init == true) {
		p_odom.pos.x = (*msg).pose.pose.position.x;
		p_odom.pos.y = (*msg).pose.pose.position.y;
		p_odom.yaw   = yaw;
		odom.yaw     = yaw;
		return;
	}

	// Integrate odometry pose to convert to the map frame
	odom.pos.x += (*msg).pose.pose.position.x - p_odom.pos.x;
	odom.pos.y += (*msg).pose.pose.position.y - p_odom.pos.y;
	odom.pos.z = 0;
	odom.roll  = 0;
	odom.pitch = 0;
	odom.yaw   = yaw;

	// Save current odometry pose to use in the next iteration
	p_odom.pos.x = (*msg).pose.pose.position.x;
	p_odom.pos.y = (*msg).pose.pose.position.y;
	p_odom.yaw   = yaw;
}

void wildSLAM_ros::SLAMNode::callbackFct(
    const sensor_msgs::ImageConstPtr&            depth_image,
    const vision_msgs::Detection2DArrayConstPtr& dets)
{
	// Declaration of the arrays that will constitute the SLAM observations
	std::vector<int>    labels;
	std::vector<double> bearings;
	std::vector<double> depths;

	// Loop over all the bounding box detections
	for (size_t i = 0; i < (*dets).detections.size(); i++) {
		// Load a single bounding box detection
		vision_msgs::BoundingBox2D m_bbox = (*dets).detections[i].bbox;

		// Calculate the depth of the detected object
		float depth =
		    computeDepth(*depth_image, m_bbox.center.x - m_bbox.size_x / 2,
		                 m_bbox.center.y - m_bbox.size_y / 2,
		                 m_bbox.center.x + m_bbox.size_x / 2,
		                 m_bbox.center.y + m_bbox.size_y / 2);

		// Check if the calculated depth is valid
		if (depth == -1)
			continue;

		// Calculate the correspondent bearing observations
		double column  = m_bbox.center.x;
		double bearing = (-h_fov / img_width) * (img_width / 2 - column);

		// Insert the measures in the observations arrays
		labels.push_back((*dets).detections[i].results[0].id);
		depths.push_back(depth);
		bearings.push_back(bearing);
	}

	if (init == true && bearings.size() > 1) {
		// Initialize the localizer and get first particles distribution
		(*localizer).init(Pose<double>(0, 0, 0, 0, 0, odom.yaw));
		Pose<double> robot_pose = (*localizer).getPose();

		// Initialize the mapper2D
		(*mapper2D).init(robot_pose, bearings, depths, labels);

		// Get first map2D
		map2D = (*mapper2D).getMap();

		init = false;
	}
	else if (init == false) {
		// Execute the localization procedure
		(*localizer).process(odom, bearings, depths, map2D);
		Pose<double>             robot_pose = (*localizer).getPose();
		geometry_msgs::PoseArray poses      = (*localizer).getPoseArray();
		tf::Transform            cam2map    = (*localizer).getTf();

		// Execute the 2D map estimation
		(*mapper2D).process(robot_pose, bearings, depths, cam2map, labels);

		// Get the curretn 2D map
		map2D = (*mapper2D).getMap();

		// Execute the 3D map estimation
		float* depths = (float*)(&(*depth_image).data[0]);
		(*mapper3D).process(depths, *dets);
		// Publish 3D point clouds
		publish3DRawMap((*depth_image).header);
		publish3DTrunkMap((*depth_image).header);

		// Publish the 2D map and particle filter
		publish2DMap((*depth_image).header, robot_pose);
		poses.header          = (*depth_image).header;
		poses.header.frame_id = "map";
		particle_publisher.publish(poses);

		// Publish cam-to-world tf::Transform
		static tf::TransformBroadcaster br;
		br.sendTransform(
		    tf::StampedTransform(cam2map, poses.header.stamp, "map", "cam"));
	}
}

double wildSLAM_ros::SLAMNode::computeDepth(const sensor_msgs::Image& depth_img,
                                            const int& xmin, const int& ymin,
                                            const int& xmax, const int& ymax)
{
	// Declare array with all the disparities computed
	float* depths = (float*)(&(depth_img).data[0]);

	double range_min = 0.01;
	double range_max = 10.0;

	std::vector<float> depth_array;
	for (int x = xmin; x < xmax; x++) {
		for (int y = ymin; y < ymax; y++) {
			int idx = x + depth_img.width * y;

			// Fill the depth array with the values of interest
			if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
			    depths[idx] < range_max)
				depth_array.push_back(depths[idx]);
		}
	}

	// compute median of all observations
	size_t n_depths = depth_array.size();
	if (n_depths > 0) {
		std::sort(depth_array.begin(), depth_array.end());
		if (n_depths % 2 == 0)
			return (depth_array[n_depths / 2 - 1] + depth_array[n_depths / 2]) / 2;
		else
			return depth_array[n_depths / 2];
	}
	else
		return -1;
}
