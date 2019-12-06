#include "../include/mapper/mapper.hpp"

Estimator::Estimator(const Parameters& params, LandmarkProcessor* lprocessor)
    : params(params), lprocessor(lprocessor)
{
}

void Estimator::process(const std::vector<Pose<double>>& robot_poses,
                        const std::vector<int>&          index)
{
	int inc = params.mapper_inc;

	std::vector<Landmark<double>> final_l;
	for (size_t i = 0; i < index.size(); i++) {
		int k = index[i];
		if ((*lprocessor).landmarks[k].image_pos.size() < inc)
			continue;

		Landmark<double> l = (*lprocessor).landmarks[k];

		int it = l.image_pos.size();

		Point<double> X_prev = l.image_pos[it - inc];
		Point<double> X_curr = l.image_pos[it];
		Pose<double>  p_pos  = robot_poses[l.ptr[it] - inc];
		Pose<double>  c_pos  = robot_poses[l.ptr[it]];

		l.estimations = (*lprocessor).pf[k].process(c_pos, p_pos, X_curr, X_prev);
		l.standardDev();

		(*lprocessor).landmarks[k] = l;
	}
}
