#include "../include/wildSLAM_ros.hpp"

void wildSLAM_ros::SLAMNode::odomListener(const nav_msgs::OdometryConstPtr& msg)
{
	// Convert odometry msg to pose msg
	tf::Pose            pose;
	geometry_msgs::Pose odom_pose = (*msg).pose.pose;
	tf::poseMsgToTF(odom_pose, pose);

	// Check if yaw is NaN
	float yaw = tf::getYaw(pose.getRotation());
	if (yaw != yaw)
		yaw = 0;

	// If it is the first iteration - initialize the Pose
	// relative to the previous frame
	if (init == true) {
		p_odom.x   = (*msg).pose.pose.position.x;
		p_odom.y   = (*msg).pose.pose.position.y;
		p_odom.yaw = yaw;
		odom.yaw   = yaw;
		return;
	}

	// Integrate odometry pose to convert to the map frame
	odom.x += (*msg).pose.pose.position.x - p_odom.x;
	odom.y += (*msg).pose.pose.position.y - p_odom.y;
	odom.z     = 0;
	odom.roll  = 0;
	odom.pitch = 0;
	odom.yaw   = yaw;

	// Save current odometry pose to use in the next iteration
	p_odom.x   = (*msg).pose.pose.position.x;
	p_odom.y   = (*msg).pose.pose.position.y;
	p_odom.yaw = yaw;
}

void wildSLAM_ros::SLAMNode::callbackFct(
    const sensor_msgs::ImageConstPtr&            left_image,
    const sensor_msgs::ImageConstPtr&            depth_image,
    const vision_msgs::Detection2DArrayConstPtr& dets)
{
	// Declaration of the arrays that will constitute the SLAM observations
	std::vector<int>   labels;
	std::vector<float> bearings;
	std::vector<float> depths;

	// Loop over all the bounding box detections
	for (size_t i = 0; i < (*dets).detections.size(); i++) {
		// Load a single bounding box detection
		vision_msgs::BoundingBox2D m_bbox = (*dets).detections[i].bbox;

		// Calculate the bearing and depth of the detected object
		float depth;
		float bearing;
		computeObsv(*depth_image, m_bbox.center.x - m_bbox.size_x / 2,
		            m_bbox.center.y - m_bbox.size_y / 2,
		            m_bbox.center.x + m_bbox.size_x / 2,
		            m_bbox.center.y + m_bbox.size_y / 2, depth, bearing);

		// Check if the calculated depth is valid
		if (depth == -1)
			continue;

		// Insert the measures in the observations arrays
		labels.push_back((*dets).detections[i].results[0].id);
		depths.push_back(depth);
		bearings.push_back(bearing);
	}

	if (init == true && bearings.size() > 1) {
		// Initialize the localizer and get first particles distribution
		(*localizer).init(pose6D(0, 0, 0, 0, 0, odom.yaw));
		pose6D robot_pose = (*localizer).getPose();

		// Initialize the mapper2D
		(*mapper2D).init(robot_pose, bearings, depths, labels);

		// Initialize the mapper3D
		(*mapper3D).init();

		// Get first cam2map
		map2D = (*mapper2D).getMap();

		init = false;
	}
	else if (init == false) {
		// Convert ROS image to OpenCV image
		cv::Mat m_img =
		    cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8)
		        ->image;

		float* raw_depths = (float*)(&(*depth_image).data[0]);

		// Execute the localization procedure
		(*localizer).process(odom, bearings, depths, map2D, raw_depths, *dets);
		pose6D robot_pose = (*localizer).getPose();

		// Execute the 3D map estimation
#if MAP3D == 1
    // User chose 3D trunk map
		(*mapper3D).trunkMap(trunk_octree, raw_depths, m_img, robot_pose, *dets);
#else
    // User chose 3D feature map
		std::vector<Feature> features;
		featureExtract(m_img, features);
		(*mapper3D).featureMap(feature_octree, raw_depths, m_img, robot_pose,
		                       features);
#endif
		// Publish 3D point map
		publish3DMap((*depth_image).header);

		// Execute the 2D map estimation
		(*mapper2D).process(robot_pose, bearings, depths, labels);
		// Get the curretn 2D map
		map2D = (*mapper2D).getMap();
		// Publish the 2D map
		publish2DMap((*depth_image).header, robot_pose, bearings, depths);

		// Convert robot pose to tf::Transform corresponding
		// to the camera to map transformation
		tf::Quaternion q;
		q.setRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);
		q.normalize();
		tf::Transform cam2map;
		cam2map.setRotation(q);
		cam2map.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, robot_pose.z));

		// Convert wildSLAM pose to ROS pose and publish it
		geometry_msgs::PoseStamped pose;
		pose.header             = (*depth_image).header;
		pose.header.frame_id    = "map";
		pose.pose.position.x    = robot_pose.x;
		pose.pose.position.y    = robot_pose.y;
		pose.pose.position.z    = robot_pose.z;
		pose.pose.orientation.x = q.x();
		pose.pose.orientation.y = q.y();
		pose.pose.orientation.z = q.z();
		pose.pose.orientation.w = q.w();
		pose_publisher.publish(pose);

		// Publish cam-to-map tf::Transform
		static tf::TransformBroadcaster br;
		br.sendTransform(
		    tf::StampedTransform(cam2map, pose.header.stamp, "map", "cam"));

#ifdef DEBUG
		// Publish all poses for DEBUG
		std::vector<pose6D> poses;
		(*localizer).getParticles(poses);
		geometry_msgs::PoseArray ros_poses;
		ros_poses.header          = (*depth_image).header;
		ros_poses.header.frame_id = "map";
		for (size_t i = 0; i < poses.size(); i++) {
			tf::Quaternion q;
			q.setRPY(poses[i].roll, poses[i].pitch, poses[i].yaw);
			q.normalize();

			geometry_msgs::Pose m_pose;
			m_pose.position.x    = poses[i].x;
			m_pose.position.y    = poses[i].y;
			m_pose.position.z    = poses[i].z;
			m_pose.orientation.x = q.x();
			m_pose.orientation.y = q.y();
			m_pose.orientation.z = q.z();
			m_pose.orientation.w = q.w();

			ros_poses.poses.push_back(m_pose);
		}
		poses_publisher.publish(ros_poses);
#endif
	}
}

void wildSLAM_ros::SLAMNode::computeObsv(const sensor_msgs::Image& depth_img,
                                         const int& xmin, const int& ymin,
                                         const int& xmax, const int& ymax,
                                         float& depth, float& bearing)
{
	// Declare array with all the disparities computed
	float* depths = (float*)(&(depth_img).data[0]);

	// Set minimum and maximum depth values to consider
	float range_min = 0.01;
	float range_max = 10.0;

	std::map<float, float> dtheta;
	for (int i = xmin; i < xmax; i++) {
		for (int j = ymin; j < ymax; j++) {
			int idx = i + depth_img.width * j;

			// Fill the depth array with the values of interest
			if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
			    depths[idx] < range_max) {
				float x         = depths[idx];
				float y         = -(float)(i - cx) * (x / fx);
				float m_depth   = sqrt(pow(x, 2) + pow(y, 2));
				dtheta[m_depth] = atan2(y, x);
			}
		}
	}

	// compute minimum of all observations
	size_t n_depths = dtheta.size();
	if (n_depths > 0) {
		depth   = dtheta.begin()->first;
		bearing = dtheta.begin()->second;
	}
	else {
		depth   = -1;
		bearing = -1;
	}
}

void wildSLAM_ros::SLAMNode::featureExtract(cv::Mat               in,
                                            std::vector<Feature>& out)
{
	// Array to store the features
	std::vector<cv::KeyPoint> kpts;
	// String to store the type of feature
	std::string type;

	// Perform feature extraction using one of the following
	// feature detectors
	if (STAR_ == 1) {
#ifdef DEBUG
		std::cout << "Using STAR feature extractor..." << std::endl;
#endif
		type      = "star";
		auto star = cv::xfeatures2d::StarDetector::create(32);
		star->detect(in, kpts);
	}
	else if (BRISK_ == 1) {
#ifdef DEBUG
		std::cout << "Using BRISK feature extractor..." << std::endl;
#endif
		type       = "brisk";
		auto brisk = cv::BRISK::create();
		brisk->detect(in, kpts);
	}
	else if (FAST_ == 1) {
#ifdef DEBUG
		std::cout << "Using FAST feature extractor..." << std::endl;
#endif
		type      = "fast";
		auto fast = cv::FastFeatureDetector::create();
		fast->detect(in, kpts);
	}
	else if (ORB_ == 1) {
#ifdef DEBUG
		std::cout << "Using ORB feature extractor..." << std::endl;
#endif
		type     = "orb";
		auto orb = cv::ORB::create();
		orb->detect(in, kpts);
	}
	else if (KAZE_ == 1) {
#ifdef DEBUG
		std::cout << "Using KAZE feature extractor..." << std::endl;
#endif
		type      = "kaze";
		auto kaze = cv::KAZE::create();
		kaze->detect(in, kpts);
	}
	else if (AKAZE_ == 1) {
#ifdef DEBUG
		std::cout << "Using AKAZE feature extractor..." << std::endl;
#endif
		type       = "akaze";
		auto akaze = cv::AKAZE::create();
		akaze->detect(in, kpts);
	}

	// Draw features into the output image
	cv::Mat out_img;
	cv::drawKeypoints(in, kpts, out_img);

	// Show (or not) the feature extraction result
	if (IMSHOW == 1) {
		cv::imshow("Feature extraction", out_img);
		cv::waitKey(0);
	}

	// Save features in the output array
	for (size_t i = 0; i < kpts.size(); i++) {
		Feature m_ft(kpts[i].pt.x, kpts[i].pt.y, type);
		out.push_back(m_ft);
	}
}
