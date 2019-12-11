#include "../include/mapper/mapper.hpp"

Estimator::Estimator(const Parameters& params, LandmarkProcessor* lprocessor)
    : params(params), lprocessor(lprocessor)
{
}

void Estimator::control()
{
	std::vector<Point<double>> pts_right;
	std::vector<Point<double>> pts_left;
	for (size_t i = 0; i < (*lprocessor).landmarks.size(); i++) {
		Landmark<double> l = (*lprocessor).landmarks[i];

		if (l.image_pos[0].x > params.width / 2)
			pts_right.push_back(l.world_pos);
		else
			pts_left.push_back(l.world_pos);
	}

	vine_right = Line<double>(pts_right);
	vine_left  = Line<double>(pts_left);
}

void Estimator::process(const std::vector<Pose<double>>& robot_poses,
                        const std::vector<int>&          index)
{
	if (params.type == "pf")
		pfPrediction(robot_poses, index);
	else
		kfPrediction(robot_poses, index);
}

void Estimator::pfPrediction(const std::vector<Pose<double>>& robot_poses,
                             const std::vector<int>&          index)
{
	int inc = params.mapper_inc;

	for (size_t i = 0; i < index.size(); i++) {
		int k = index[i];
		if ((*lprocessor).landmarks[k].image_pos.size() - 1 < inc)
			continue;

		Landmark<double> l = (*lprocessor).landmarks[k];

		int it = l.image_pos.size() - 1;

		Point<double> X_prev = l.image_pos[it - inc];
		Point<double> X_curr = l.image_pos[it];
		Pose<double>  p_pos  = robot_poses[l.ptr[it - 1] - inc];
		Pose<double>  c_pos  = robot_poses[l.ptr[it - 1]];

		Line<double> vine;
		if (l.image_pos[0].x > params.width / 2)
			vine = vine_right;
		else
			vine = vine_left;

		l.estimations =
		    (*lprocessor).pf[k].process(c_pos, p_pos, X_curr, X_prev, vine);
		l.worldPos();
		l.standardDev();

		(*lprocessor).landmarks[k] = l;
	}
}

void Estimator::kfPrediction(const std::vector<Pose<double>>& robot_poses,
                             const std::vector<int>&          index)
{
	int inc = params.mapper_inc;

	for (size_t i = 0; i < index.size(); i++) {
		int k = index[i];
		if ((*lprocessor).landmarks[k].image_pos.size() - 1 < inc)
			continue;

		Landmark<double> l = (*lprocessor).landmarks[k];

		int it = l.image_pos.size() - 1;

		Pose<double>  p_pos = robot_poses[l.ptr[it - 1] - l.ptr[it - 1 - inc]];
		Pose<double>  c_pos = robot_poses[l.ptr[it - 1]];
		Point<double> X     = processObsv(l, it, c_pos - p_pos);

		VectorXd z(2, 1);
		z << sqrt((X.x * X.x) + (X.y * X.y)), atan2(X.y, X.x);

		/* kalman filter */
		Pose<double> s = p_pos;
		std::cout << (*lprocessor).kf.size() << "," << k << std::endl;
		(*lprocessor).kf[k].process(s.eig(), z);
		VectorXd TH = (*lprocessor).kf[k].getState();

		Point<double> res(TH(0), TH(1));
		(*lprocessor).landmarks[k].estimations.push_back(res);
		(*lprocessor).landmarks[k].world_pos = res;
		std::cout << (*lprocessor).landmarks[k].world_pos << std::endl;
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
