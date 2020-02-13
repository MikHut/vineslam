#include "../include/mapper/mapper.hpp"

Estimator::Estimator(const Parameters& params, LandmarkProcessor* lprocessor)
    : params(params), lprocessor(lprocessor)
{
}

void Estimator::process(const std::vector<Pose<double>>& robot_poses,
                        const std::vector<int>&          index)
{
	std::vector<Pose<double>> filtered_poses;
	filterXYTheta(robot_poses, filtered_poses);
	/* predict(filtered_poses); */
	predict(filtered_poses, index);
}

void Estimator::predict(const std::vector<Pose<double>>& robot_poses,
                        const std::vector<int>&          index)
{
	int inc = params.mapper_inc;

	for (size_t i = 0; i < index.size(); i++) {
		int k = index[i];
		if ((*lprocessor).landmarks[k].image_pos.size() < inc)
			continue;

		Landmark<double> l = (*lprocessor).landmarks[k];

		/* initialize landmark on map if it does not exist yet */
		if (l.image_pos.size() == params.init_dim && kf.find(k) == kf.end()) {
			initLandmark(robot_poses, k);
			continue;
		}
		else if (l.image_pos.size() < params.init_dim || kf.find(k) == kf.end())
			continue;

		int it = l.image_pos.size() - 1;

		/* calculate the bearing-only observation */
		Pose<double>  p_pos = robot_poses[l.ptr[it] - l.ptr[it - inc]];
		Pose<double>  c_pos = robot_poses[l.ptr[it]];
		Point<double> X     = processObsv(l, it, c_pos - p_pos);

		VectorXd z(2, 1);
		z << sqrt((X.x * X.x) + (X.y * X.y)), atan2(X.y, X.x);

		/* kalman filter */
		Pose<double> s = p_pos;
		kf[k].process(s.eig(), z);
		VectorXd TH = kf[k].getState();

		Point<double> res(TH(0), TH(1));
		(*lprocessor).landmarks[k].estimations.push_back(res);
		(*lprocessor).landmarks[k].world_pos = res;
	}
}

Point<double> Estimator::processObsv(const Landmark<double>& l, const int& it,
                                     const Pose<double>& delta_p)
{
	int comp = params.filter_window;
	int inc  = params.mapper_inc;

	Point<double> X_prev = l.image_pos[it - inc];
	Point<double> X_curr = l.image_pos[it];

	Line<double> l_prev = (*lprocessor).computeLine(X_prev);
	Line<double> l_proj =
	    (*lprocessor).projectLine(X_curr, delta_p.pos, delta_p.theta);
	Point<double> X = l_prev.intercept(l_proj);

	return X;
}
