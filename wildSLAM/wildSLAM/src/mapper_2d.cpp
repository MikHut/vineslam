#include "mapper_2d.hpp"

Mapper2D::Mapper2D(const std::string& config_path) : config_path(config_path)
{
	YAML::Node config = YAML::LoadFile(config_path.c_str());
	fx                = config["camera_info"]["fx"].as<float>();
	baseline          = config["camera_info"]["baseline"].as<float>();
	delta_d           = config["camera_info"]["delta_d"].as<float>();
}

void Mapper2D::init(pose6D& pose, const std::vector<float>& bearings,
                    const std::vector<float>& depths,
                    const std::vector<int>&   labels)
{
	int       n_obsv    = bearings.size();
	ellipse2D robot_std = pose.getDist();

	// Compute initial covariance matrix
	// - proportional to the pose signal and the distance to the robot
	for (int i = 0; i < n_obsv; i++) {
		// Construct the observations vector
		VectorXd z(2, 1);
		z << depths[i], bearings[i];

		// Calculate
		// - the initial estimation of the landmark
		// - the initial observation covariance of the landmark
		float   th = bearings[i] + pose.yaw;
		point3D X(pose.x + depths[i] * cos(th), pose.y + depths[i] * sin(th), 0.);

		// Push back a Kalman Filter object for the respective landmark
		KF kf(X.toEig2D(), pose.toEig2D(), robot_std.toEig(), z, config_path);
		filters.push_back(kf);

		// Insert the landmark on the map, with a single observation
		// and get the correspondent standard deviation
		point3D   pos(X.x, X.y, 0.);
		ellipse2D std = filters[filters.size() - 1].getStdev();
		if (map.empty() == 0)
			map[map.rbegin()->first + 1] = Landmark<float>(pos, std, labels[i]);
		else
			map[1] = Landmark<float>(pos, std);
	}
}

void Mapper2D::process(pose6D& pose, const std::vector<float>& bearings,
                       const std::vector<float>& depths,
                       const std::vector<int>&   labels)
{
	// Compute local map on robot's referential frame
	std::vector<point3D> l_map = local_map(pose, bearings, depths);
	// Convert local map to bearings and depths
	std::vector<float> bearings_(bearings.size());
	std::vector<float> depths_(depths.size());
	for (size_t i = 0; i < l_map.size(); i++) {
		depths_[i]   = sqrt(pow(l_map[i].x, 2) + pow(l_map[i].y, 2));
		bearings_[i] = atan2(l_map[i].y, l_map[i].x) - pose.yaw;
	}
	// Estimate global map
	predict(pose, bearings_, depths_, labels);
}

std::vector<point3D> Mapper2D::local_map(pose6D&                   pose,
                                         const std::vector<float>& bearings,
                                         const std::vector<float>& depths)
{
	std::vector<point3D> landmarks;

	// Convert 6-DOF pose to homogenous transformation
	std::vector<float> Rot;
	point3D            trans = pose.getXYZ();
	pose.toRotMatrix(Rot);

	for (size_t i = 0; i < bearings.size(); i++) {
		// Calculate the estimation of the landmark position on
		// camera's referential frame
		float   th = bearings[i];
		point3D X_cam(depths[i] * cos(th), depths[i] * sin(th), 0.);

		// Convert landmark to map's referential frame
		point3D X_map;
		X_map.x = X_cam.x * Rot[0] + X_cam.y * Rot[1] + X_cam.z * Rot[2] + trans.x;
		X_map.y = X_cam.x * Rot[3] + X_cam.y * Rot[4] + X_cam.z * Rot[5] + trans.y;
		X_map.z = X_cam.x * Rot[6] + X_cam.y * Rot[7] + X_cam.z * Rot[8] + trans.z;

		// Convert landmark to robot's referential frame and insert
		// on array of landmarks
		point3D X_robot = X_map - pose.getXYZ();
		landmarks.push_back(X_robot);
	}

	return landmarks;
}

void Mapper2D::predict(pose6D& pose, const std::vector<float>& bearings,
                       const std::vector<float>& depths,
                       const std::vector<int>&   labels)
{
	int       n_obsv    = bearings.size();
	ellipse2D robot_std = pose.getDist();

	for (int i = 0; i < n_obsv; i++) {
		// Calculate the landmark position based on the ith observation
		float   th = normalizeAngle(bearings[i] + pose.yaw);
		point3D X(pose.x + depths[i] * cos(th),
		          pose.y + depths[i] * sin(th), 0.);
		// Construct the observations vector
		VectorXd z(2, 1);
		z << depths[i], bearings[i];

		// Check if the landmark already exists in the map
		int landmark_id = findCorr(X, pose.getXYZ());
		// If not, initialize the landmark on the map, as well as the
		// correspondent Kalman Filter
		if (landmark_id < 0) {
			Eigen::MatrixXd R(2, 2);

			// Initialize the Kalman Filter
			KF kf(X.toEig2D(), pose.toEig2D(), robot_std.toEig(), z,
			      config_path);
			filters.push_back(kf);

			// Insert the landmark on the map, with a single observation
			ellipse2D stdev              = filters[filters.size() - 1].getStdev();
			map[map.rbegin()->first + 1] = Landmark<float>(X, stdev, labels[i]);
		}
		// If so, update the landmark position estimation using a Kalman
		// Filter call
		else {
			// Invocate the Kalman Filter
			filters[landmark_id - 1].process(pose.toEig2D(), robot_std.toEig(),
			                                 z);
			// Get the state vector and the standard deviation associated
			// with the estimation
			point3D   X_out = filters[landmark_id - 1].getState();
			ellipse2D stdev = filters[landmark_id - 1].getStdev();

			// Update the estimation on the map
			map[landmark_id] = Landmark<float>(X_out, stdev, labels[i]);
		}
	}
}

int Mapper2D::findCorr(const point3D& l_pos, const point3D& r_pos)
{
	int   best_correspondence = -1;
	float best_aprox          = 0.5;
	for (auto m_map : map) {
		// Compute the euclidean distance between the observation
		// and the corrent landmark on the map
		float dist = l_pos.distanceXY(m_map.second.pos);

		// Return the id of the landmark, if a correspondence is found
		if (dist < best_aprox) {
			best_correspondence = m_map.first;
			best_aprox          = dist;
		}
	}

	return best_correspondence;
}

std::map<int, Landmark<float>> Mapper2D::getMap() const
{
	return map;
}
