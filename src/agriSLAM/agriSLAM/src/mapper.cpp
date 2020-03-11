#include "mapper.hpp"

Mapper::Mapper(const Parameters& params) : params(params) {}

void Mapper::init(const Pose<double>& pose, const std::vector<double>& bearings,
                  const std::vector<double>&       depths,
                  const std::vector<SemanticInfo>& info)
{
	int           n_obsv = bearings.size();
	Point<double> pos    = pose.pos;

	// Compute initial covariance matrix
	// - proportional to the pose signal and the distance to the robot
	for (int i = 0; i < n_obsv; i++) {
		// Construct the observations vector
		VectorXd z(2, 1);
		z << depths[i], bearings[i];

		// Calculate
		// - the initial estimation of the landmark
		// - the initial observation covariance of the landmark
		double          th = bearings[i] + pose.yaw;
		Point<double>   X(pose.pos.x + depths[i] * cos(th),
                    pose.pos.y + depths[i] * sin(th));
		Eigen::MatrixXd R(2, 2);
		R(0, 0) = dispError(depths[i]) * std::fabs(cos(bearings[i]));
		R(1, 1) = dispError(depths[i]) * std::fabs(sin(bearings[i]));
		R(0, 1) = 0;
		R(1, 0) = 0;

		// Push back a Kalman Filter object for the respective landmark
		KF kf(X.eig_2d(), pos.eig_2d(), z, R, params);
		filters.push_back(kf);

		// Insert the landmark on the map, with a single observation
		// and get the correspondent standard deviation
		Point<double>   pos(X.x, X.y);
		Ellipse<double> std = filters[filters.size() - 1].getStdev();
		if (map.empty() == 0)
			map[map.rbegin()->first + 1] = Landmark<double>(pos, std, info[i]);
		else
			map[1] = Landmark<double>(pos, std);
	}
}

void Mapper::process(const Pose<double>&              pose,
                     const std::vector<double>&       bearings,
                     const std::vector<double>&       depths,
                     const tf::Transform&             cam2map,
                     const std::vector<SemanticInfo>& info)
{
	// Compute local map on robot's referential frame
	std::vector<Point<double>> l_map = local_map(pose, bearings, depths, cam2map);
	// Convert local map to bearings and depths
	std::vector<double> bearings_(bearings.size());
	std::vector<double> depths_(depths.size());
	for (size_t i = 0; i < l_map.size(); i++) {
		depths_[i]   = sqrt(pow(l_map[i].x, 2) + pow(l_map[i].y, 2));
		bearings_[i] = atan2(l_map[i].y, l_map[i].x) - pose.yaw;
	}
	// Estimate global map
	predict(pose, bearings_, depths_, info);
}

std::vector<Point<double>>
Mapper::local_map(const Pose<double>& pose, const std::vector<double>& bearings,
                  const std::vector<double>& depths,
                  const tf::Transform&       cam2world)
{
	std::vector<Point<double>> landmarks;

	// Decompose homogeneous transformation into
	// a rotation matrix and a translation vector
	tf::Matrix3x3 R = cam2world.getBasis();
	tf::Vector3   t = cam2world.getOrigin();

	for (size_t i = 0; i < bearings.size(); i++) {
		// Calculate the estimation of the landmark position on
		// camera's referential frame
		double        th = bearings[i];
		Point<double> X_cam(depths[i] * cos(th), depths[i] * sin(th), 0);

		// Convert landmark to world's referential frame
		Point<double> X_world;
		X_world.x = X_cam.x * R[0].getX() + X_cam.y * R[0].getY() +
		            X_cam.z * R[0].getZ() + t.getX();
		X_world.y = X_cam.x * R[1].getX() + X_cam.y * R[1].getY() +
		            X_cam.z * R[1].getZ() + t.getY();
		X_world.z = X_cam.x * R[2].getX() + X_cam.y * R[2].getY() +
		            X_cam.z * R[2].getZ() + t.getZ();

		// Convert landmark to robot's referential frame and insert
		// on array of landmarks
		Point<double> X_robot = X_world - pose.pos;
		landmarks.push_back(X_robot);
	}

	return landmarks;
}

void Mapper::predict(const Pose<double>&              pose,
                     const std::vector<double>&       bearings,
                     const std::vector<double>&       depths,
                     const std::vector<SemanticInfo>& info)
{
	int           n_obsv   = bearings.size();
	double        pose_std = 0.2;
	Point<double> pos      = pose.pos;

	for (int i = 0; i < n_obsv; i++) {
		// Calculate the landmark position based on the ith observation
		double        th = normalizeAngle(bearings[i] + pose.yaw);
		Point<double> X(pos.x + depths[i] * cos(th), pos.y + depths[i] * sin(th));
		// Construct the observations vector
		VectorXd z(2, 1);
		z << depths[i], bearings[i];

		// Check if the landmark already exists in the map
		int landmark_id = findCorr(X, pos);
		// If not, initialize the landmark on the map, as well as the
		// correspondent Kalman Filter
		if (landmark_id < 0) {
			Eigen::MatrixXd R(2, 2);

			// Calculate the initial covariance
			R(0, 0) = dispError(depths[i]) * std::fabs(cos(bearings[i])) + (pose_std);
			R(1, 1) = dispError(depths[i]) * std::fabs(sin(bearings[i])) + (pose_std);
			R(0, 1) = ((R(0, 0) + R(1, 1)) / 2.0) * tan(bearings[i]);
			R(1, 0) = ((R(0, 0) + R(1, 1)) / 2.0) * tan(bearings[i]);

			// Initialize the Kalman Filter
			KF kf(X.eig_2d(), pos.eig_2d(), z, R, params);
			filters.push_back(kf);

			// Insert the landmark on the map, with a single observation
			Ellipse<double> std          = filters[filters.size() - 1].getStdev();
			map[map.rbegin()->first + 1] = Landmark<double>(X, std, info[i]);
		}
		// If so, update the landmark position estimation using a Kalman
		// Filter call
		else {
			// Invocate the Kalman Filter
			filters[landmark_id - 1].process(pos.eig_2d(), z);
			// Get the state vector and the standard deviation associated
			// with the estimation
			Point<double>   X_out = filters[landmark_id - 1].getState();
			Ellipse<double> stdev = filters[landmark_id - 1].getStdev();

			// Update the estimation on the map
			map[landmark_id] = Landmark<double>(X_out, stdev, info[i]);
		}
	}
}

int Mapper::findCorr(const Point<double>& l_pos, const Point<double>& r_pos)
{
	int best_correspondence = -1;
  double best_aprox = 0.5;
	for (auto m_map : map) {
    // Compute the euclidean distance between the observation
    // and the corrent landmark on the map
		double dist = sqrt(pow(l_pos.x - m_map.second.pos.x, 2) +
		                   pow(l_pos.y - m_map.second.pos.y, 2));

		// Return the id of the landmark, if a correspondence is found
		if (dist < best_aprox) {
			best_correspondence = m_map.first;
			best_aprox          = dist;
		}
	}

	return best_correspondence;
}

std::map<int, Landmark<double>> Mapper::getMap() const
{
	return map;
}
