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

void wildSLAM_ros::SLAMNode::publish3DRawMap(const std_msgs::Header& header)
{
	// Get the raw point cloud to publish
	OcTreeT *octree = (*mapper3D).getRawPointCloud();

	visualization_msgs::MarkerArray octomapviz;
	// each array stores all cubes of a different size, one for each depth level:
	octomapviz.markers.resize((*octree).getTreeDepth() + 1);

  int j = 0;
	for (OcTreeT::iterator it  = (*octree).begin(),
	                       end = (*octree).end();
	     it != end; ++it) {

		//if ((*it).isColorSet()) {
		if ((*octree).isNodeOccupied(*it)) {
			double size = it.getSize();
			double x    = it.getX();
			double y    = it.getY();
			double z    = it.getZ();

			std_msgs::ColorRGBA _color;
			_color.r = (*it).getColor().r / 255.;
			_color.g = (*it).getColor().g / 255.;
			_color.b = (*it).getColor().b / 255.;
			_color.a = 1.0;

			unsigned idx = it.getDepth();
			assert(idx < octomapviz.markers.size());

			geometry_msgs::Point cubeCenter;
			cubeCenter.x = x;
			cubeCenter.y = y;
			cubeCenter.z = z;

			octomapviz.markers[idx].points.push_back(cubeCenter);
			octomapviz.markers[idx].colors.push_back(_color);
		}
	}

	for (unsigned i = 0; i < octomapviz.markers.size(); ++i) {
		double size = (*octree).getNodeSize(i);

		octomapviz.markers[i].header.frame_id = "map";
		octomapviz.markers[i].header.stamp    = header.stamp;
		octomapviz.markers[i].ns              = "map";
		octomapviz.markers[i].id              = i;
		octomapviz.markers[i].type    = visualization_msgs::Marker::CUBE_LIST;
		octomapviz.markers[i].scale.x = size;
		octomapviz.markers[i].scale.y = size;
		octomapviz.markers[i].scale.z = size;

		if (octomapviz.markers[i].points.size() > 0)
			octomapviz.markers[i].action = visualization_msgs::Marker::ADD;
		else
			octomapviz.markers[i].action = visualization_msgs::Marker::DELETE;
	}

	map3D_raw_publisher.publish(octomapviz);
}

void wildSLAM_ros::SLAMNode::publish3DTrunkMap(const std_msgs::Header& header)
{
	// Get the raw point cloud to publish
	OcTreeT* octree = (*mapper3D).getTrunkPointCloud();

	visualization_msgs::MarkerArray octomapviz;
	// each array stores all cubes of a different size, one for each depth level:
	octomapviz.markers.resize((*octree).getTreeDepth() + 1);

	for (OcTreeT::iterator it  = (*octree).begin((*octree).getTreeDepth()),
	                       end = (*octree).end();
	     it != end; ++it) {

		if ((*octree).isNodeOccupied(*it)) {
		//if ((*it).isColorSet()) {
			double size = it.getSize();
			double x    = it.getX();
			double y    = it.getY();
			double z    = it.getZ();

			std_msgs::ColorRGBA _color;
			_color.r = (*it).getColor().r / 255.;
			_color.g = (*it).getColor().g / 255.;
			_color.b = (*it).getColor().b / 255.;
			_color.a = 1.0;

			unsigned idx = it.getDepth();
			assert(idx < octomapviz.markers.size());

			geometry_msgs::Point cubeCenter;
			cubeCenter.x = x;
			cubeCenter.y = y;
			cubeCenter.z = z;

			octomapviz.markers[idx].points.push_back(cubeCenter);
			octomapviz.markers[idx].colors.push_back(_color);
		}
	}

	for (unsigned i = 0; i < octomapviz.markers.size(); ++i) {
		double size = (*octree).getNodeSize(i);

		octomapviz.markers[i].header.frame_id = "map";
		octomapviz.markers[i].header.stamp    = header.stamp;
		octomapviz.markers[i].ns              = "map";
		octomapviz.markers[i].id              = i;
		octomapviz.markers[i].type    = visualization_msgs::Marker::CUBE_LIST;
		octomapviz.markers[i].scale.x = size;
		octomapviz.markers[i].scale.y = size;
		octomapviz.markers[i].scale.z = size;

		if (octomapviz.markers[i].points.size() > 0)
			octomapviz.markers[i].action = visualization_msgs::Marker::ADD;
		else
			octomapviz.markers[i].action = visualization_msgs::Marker::DELETE;
	}

	map3D_trunk_publisher.publish(octomapviz);
}
