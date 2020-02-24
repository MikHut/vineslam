#include "mapper.hpp"

Mapper::Mapper(const Parameters& params) : params(params) {}

void Mapper::init(const Pose<double>& odom, const std::vector<double>& bearings,
                  const std::vector<double>& depths)
{
	int n_obsv = bearings.size();

	// Compute initial covariance matrix
	// - proportional to the odometry signal and the distance to the robot
	for (int i = 0; i < n_obsv; i++) {
		Eigen::MatrixXd P(2, 2);

		double th = bearings[i] + odom.theta;

		// Calculate
		// - the initial estimation of the landmark
		// - the initial covariance of the landmark
		Point<double> X(odom.pos.x + depths[i] * cos(th),
		                odom.pos.y + depths[i] * sin(th));
		P(0, 0) = dispError(depths[i]) * std::fabs(cos(bearings[i]));
		P(1, 1) = dispError(depths[i]) * std::fabs(sin(bearings[i]));
		/* P(0, 1) = 0; */
		/* P(1, 0) = 0; */
		P(0, 1) = tan(bearings[i]);
		P(1, 0) = tan(bearings[i]);

		// Push back a Kalman Filter object for the respective landmark
		KF kf(X.eig(), P, params);
		filters.push_back(kf);

		// Insert the landmark on the map, with a single observation
		// and get the correspondent standard deviation
		Point<double>   pos(X.x, X.y);
		Ellipse<double> std = filters[filters.size() - 1].getStdev();
		if (map.empty() == 0)
			map[map.rbegin()->first + 1] = Landmark<double>(pos, std);
		else
			map[1] = Landmark<double>(pos, std);
	}

	ROS_INFO("Map initialized with the following landmarks:");
	for (auto m_map : map)
		std::cout << " ---> " << m_map.first << " - " << m_map.second;
}

void Mapper::process(const Pose<double>&        odom,
                     const std::vector<double>& bearings,
                     const std::vector<double>& depths)
{
	predict(odom, bearings, depths);
}

void Mapper::predict(const Pose<double>&        odom,
                     const std::vector<double>& bearings,
                     const std::vector<double>& depths)
{
	int           n_obsv   = bearings.size();
	double        k        = 0.03;
	double        a        = 0.5;
	double        odom_std = sqrt(pow(odom.pos.x, 2) + pow(odom.pos.y, 2));
	Point<double> odom_pos = odom.pos;

	for (int i = 0; i < n_obsv; i++) {
		// Calculate the landmark position based on the ith observation
		double        th = bearings[i] + odom.theta;
		Point<double> X(odom_pos.x + depths[i] * cos(th),
		                odom_pos.y + depths[i] * sin(th));

		// Check if the landmark already exists in the map
		int landmark_id = findCorr(X);
		// If not, initialize the landmark on the map, as well as the
		// correspondent Kalman Filter
		if (landmark_id < 0) {
			Eigen::MatrixXd P(2, 2);

			// Calculate the initial covariance
			P(0, 0) = dispError(depths[i]) * std::fabs(cos(bearings[i])) *
			          (odom_std * 1.01);
			P(1, 1) = dispError(depths[i]) * std::fabs(sin(bearings[i])) *
			          (odom_std * 1.01);
			/* P(0, 1) = 0; */
			/* P(1, 0) = 0; */
			P(0, 1) = tan(bearings[i]);
			P(1, 0) = tan(bearings[i]);

			// Initialize the Kalman Filter
			KF kf(X.eig(), P, params);
			filters.push_back(kf);

			// Insert the landmark on the map, with a single observation
			Ellipse<double> std          = filters[filters.size() - 1].getStdev();
			map[map.rbegin()->first + 1] = Landmark<double>(X, std);
		}
		// If so, update the landmark position estimation using a Kalman
		// Filter call
		else {
			// Construct the observations vector
			VectorXd z(2, 1);
			z << depths[i], bearings[i];

			// Invocate the Kalman Filter
			filters[landmark_id - 1].process(odom_pos.eig(), z);
			// Get the state vector and the standard deviation associated
			// with the estimation
			Point<double>   X_out = filters[landmark_id - 1].getState();
			Ellipse<double> stdev = filters[landmark_id - 1].getStdev();
			std::cout << stdev << std::endl;

			// Update the estimation on the map
			map[landmark_id] = Landmark<double>(X_out, stdev);
		}
	}
}

int Mapper::findCorr(const Point<double>& pos)
{
	for (auto m_map : map) {
		// Compute the euclidean distance between the observation and the
		// existent landmark on the map
		Point<double> p    = m_map.second.pos;
		double        dist = p.euc_dist(pos);

		// Return the id of the landmark, if a correspondence is found
		if (dist < 0.3)
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
