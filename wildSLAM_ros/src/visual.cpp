#include "../include/wildSLAM_ros.hpp"

void wildSLAM_ros::SLAMNode::publish2DMap(const std_msgs::Header& header,
                                          const Pose<double>&     pose)
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
	for (auto m_map : map2D) {
		// Draw landmark mean
		marker.id              = m_map.first;
		marker.header          = header;
		marker.header.frame_id = "map";
		marker.pose.position.x = m_map.second.pos.x;
		marker.pose.position.y = m_map.second.pos.y;
		marker.pose.position.z = 0;

		marker_array.markers.push_back(marker);

		// Draw landmark standard deviation
		tf2::Quaternion q;
		q.setRPY(0, 0, m_map.second.stdev.th);

		ellipse.id                 = m_map.first;
		ellipse.header             = header;
		ellipse.header.frame_id    = "map";
		ellipse.pose.position.x    = m_map.second.pos.x;
		ellipse.pose.position.y    = m_map.second.pos.y;
		ellipse.pose.position.z    = 0;
		ellipse.scale.x            = 3 * m_map.second.stdev.std_x;
		ellipse.scale.y            = 3 * m_map.second.stdev.std_y;
		ellipse.pose.orientation.x = q.x();
		ellipse.pose.orientation.y = q.y();
		ellipse.pose.orientation.z = q.z();
		ellipse.pose.orientation.w = q.w();

		ellipse_array.markers.push_back(ellipse);
	}

	// Draw ellipse that characterizes particles distribution
	tf2::Quaternion q;
	q.setRPY(0, 0, pose.gaussian.th);

	ellipse.id                 = map2D.size() + 1;
	ellipse.header             = header;
	ellipse.header.frame_id    = "map";
	ellipse.pose.position.x    = pose.pos.x;
	ellipse.pose.position.y    = pose.pos.y;
	ellipse.pose.position.z    = 0;
	ellipse.scale.x            = 3 * pose.gaussian.std_x;
	ellipse.scale.y            = 3 * pose.gaussian.std_y;
	ellipse.pose.orientation.x = q.x();
	ellipse.pose.orientation.y = q.y();
	ellipse.pose.orientation.z = q.z();
	ellipse.pose.orientation.w = q.w();
	ellipse.color.r            = 0.0f;
	ellipse.color.g            = 0.0f;
	ellipse.color.b            = 1.0f;
	ellipse.color.a            = 1.0f;
	ellipse_array.markers.push_back(ellipse);

	map2D_publisher.publish(marker_array);
	map2D_publisher.publish(ellipse_array);
}

void wildSLAM_ros::SLAMNode::publish3DCloud(const std_msgs::Header& header)
{
	// Get the raw point cloud to publish
	std::vector<Point<double>> in_pcl = (*mapper3D).getPointCloud();

	// Convert the point cloud to sensor_msgs::PointCloud
	sensor_msgs::PointCloud tmp_pcl;
	tmp_pcl.header          = header;
	tmp_pcl.header.frame_id = "cam";

	for (size_t i = 0; i < in_pcl.size(); i++) {
		geometry_msgs::Point32 pt;
		pt.x = in_pcl[i].z;
		pt.y = -in_pcl[i].x;
		pt.z = -in_pcl[i].y;

		tmp_pcl.points.push_back(pt);
	}

	// Convert sensor_msgs::PointCloud to sensor_msgs::PointCloud2
	sensor_msgs::PointCloud2 out_pcl;
	sensor_msgs::convertPointCloudToPointCloud2(tmp_pcl, out_pcl);

	// Publish PointCloud
	map3D_publisher.publish(out_pcl);
}
