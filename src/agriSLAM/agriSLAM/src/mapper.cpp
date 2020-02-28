#include "mapper.hpp"

Mapper::Mapper(const Parameters& params) : params(params) {}

void Mapper::init(const Pose<double>& odom, const std::vector<double>& bearings,
                  const std::vector<double>&       depths,
                  const std::vector<SemanticInfo>& info)
{
	int           n_obsv   = bearings.size();
	Point<double> odom_pos = odom.pos;

	// Compute initial covariance matrix
	// - proportional to the odometry signal and the distance to the robot
	for (int i = 0; i < n_obsv; i++) {
		// Construct the observations vector
		VectorXd z(2, 1);
		z << depths[i], bearings[i];

		// Calculate
		// - the initial estimation of the landmark
		// - the initial observation covariance of the landmark
		double          th = bearings[i] + odom.yaw;
		Point<double>   X(odom.pos.x + depths[i] * cos(th),
                    odom.pos.y + depths[i] * sin(th));
		Eigen::MatrixXd R(2, 2);
		R(0, 0) = dispError(depths[i]) * std::fabs(cos(bearings[i]));
		R(1, 1) = dispError(depths[i]) * std::fabs(sin(bearings[i]));
		R(0, 1) = 0;
		R(1, 0) = 0;

		// Push back a Kalman Filter object for the respective landmark
		KF kf(X.eig_2d(), odom_pos.eig_2d(), z, R, params);
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

void Mapper::process(const Pose<double>&        odom,
                     const std::vector<double>& bearings,
                     const std::vector<double>& depths,
                     const std::vector<SemanticInfo>& info)
{
	predict(odom, bearings, depths, info);
}

void Mapper::predict(const Pose<double>&              odom,
                     const std::vector<double>&       bearings,
                     const std::vector<double>&       depths,
                     const std::vector<SemanticInfo>& info)
{
	int           n_obsv   = bearings.size();
	double        k        = 0.03;
	double        a        = 0.5;
	double        odom_std = sqrt(pow(odom.pos.x, 2) + pow(odom.pos.y, 2));
	Point<double> odom_pos = odom.pos;

	for (int i = 0; i < n_obsv; i++) {
		// Calculate the landmark position based on the ith observation
		double        th = bearings[i] + odom.yaw;
		Point<double> X(odom_pos.x + depths[i] * cos(th),
		                odom_pos.y + depths[i] * sin(th));
		// Construct the observations vector
		VectorXd z(2, 1);
		z << depths[i], bearings[i];

		// Check if the landmark already exists in the map
		int landmark_id = findCorr(X, odom_pos);
		// If not, initialize the landmark on the map, as well as the
		// correspondent Kalman Filter
		if (landmark_id < 0) {
			Eigen::MatrixXd R(2, 2);

			// Calculate the initial covariance
			R(0, 0) = dispError(depths[i]) * std::fabs(cos(bearings[i])) *
			          (odom_std * 1.01);
			R(1, 1) = dispError(depths[i]) * std::fabs(sin(bearings[i])) *
			          (odom_std * 1.01);
			R(0, 1) = 0;
			R(1, 0) = 0;

			// Initialize the Kalman Filter
			KF kf(X.eig_2d(), odom_pos.eig_2d(), z, R, params);
			filters.push_back(kf);

			// Insert the landmark on the map, with a single observation
			Ellipse<double> std          = filters[filters.size() - 1].getStdev();
			map[map.rbegin()->first + 1] = Landmark<double>(X, std, info[i]);
		}
		// If so, update the landmark position estimation using a Kalman
		// Filter call
		else {
			// Invocate the Kalman Filter
			filters[landmark_id - 1].process(odom_pos.eig_2d(), z);
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
	for (auto m_map : map) {
		// Compute the x distance between the two landmarks and
		// check if the observation and the landmark on the map
		// are on the same same of the corridor
		double dist_x = l_pos.x - m_map.second.pos.x;
		double max_x  = 3 * m_map.second.stdev.std_x;
		double side   = (r_pos.y - l_pos.y) * (r_pos.y - m_map.second.pos.y);

		// Return the id of the landmark, if a correspondence is found
		if (std::fabs(dist_x) < std::fabs(max_x) && side > 0)
			return m_map.first;
		else
			continue;
	}

	return -1;
}

std::map<int, Landmark<double>> Mapper::getMap() const
{
	return map;
}
