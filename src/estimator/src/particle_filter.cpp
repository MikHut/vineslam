#include "../include/particle_filter.hpp"

ParticleFilter::ParticleFilter(const Parameters& params) : params(params)
{
	landm_obj = new LandmarkProcessor(params);
	prev_pose = Pose<double>(Point<double>(0, 0), 0);
}

void ParticleFilter::init()
{
	int n_particles = 1;
	weights.resize(n_particles);

	std::normal_distribution<double> dist_x(MEAN_X, STD_X);
	std::normal_distribution<double> dist_y(MEAN_Y, STD_Y);
	std::normal_distribution<double> dist_theta(MEAN_THETA, STD_THETA);

	std::default_random_engine generator;

	for (int i = 0; i < n_particles; i++) {
		Particle<double> p;
		p.id     = i;
		p.pos    = Point<double>(dist_x(generator), dist_y(generator));
		p.theta  = dist_theta(generator);
		p.weight = 1.0;

		particles.push_back(p);
	}
}

void ParticleFilter::process(const std::vector<Point<double>>& landm_pos,
                             const Pose<double>&               delta_pose)
{

#ifdef VISUALIZE
	(*landm_obj).p_map = cv::Mat(params.resolution, params.resolution, CV_8UC1,
	                             cv::Scalar(255, 255, 255));
#endif
	predict(landm_pos, delta_pose);
	updateWeights(delta_pose);
	resample();

	prev_pose.pos = prev_pose.pos + delta_pose.pos;
	prev_pose.theta += delta_pose.theta;
}

void ParticleFilter::predict(const std::vector<Point<double>>& landm_pos,
                             const Pose<double>&               delta_pose)
{
	(*landm_obj).updatePoses(landm_pos);
	(*landm_obj).matchLandmarks();

	std::normal_distribution<double> dist_x(delta_pose.pos.x, STD_X);
	std::normal_distribution<double> dist_y(delta_pose.pos.y, STD_Y);
	std::normal_distribution<double> dist_theta(delta_pose.theta, STD_THETA);

	std::default_random_engine generator;


	std::cout << "DELTA POSE" << std::endl;
	std::cout << delta_pose << std::endl;

	for (size_t i = 0; i < (*landm_obj).matches.size(); i++) {
		Match<double>              m = (*landm_obj).matches[i];
		Point<double>              avg(0, 0);
		std::vector<Point<double>> all_res;
#ifdef VISUALIZE
		(*landm_obj).plotGrid(m.p_line, 0);
#endif

		for (size_t j = 0; j < particles.size(); j++) {
			/* add gaussian noise to each particle centered on the displacement
			 * measurement */
			particles[j].pos   = Point<double>(dist_x(generator), dist_y(generator));
			particles[j].theta = dist_theta(generator);

			Particle<double> p = particles[j];
      /* p.pos.y = 0; */
      /* p.theta = 0; */

			Line<double>  pl  = m.p_line;
			Line<double>  tmp = (*landm_obj).projectLine(m, p.pos, p.theta);
			Point<double> X   = pl.intercept(tmp);

			/* calculate trunk average position estimation */
			Point<double> pt = X + prev_pose.pos;
			all_res.push_back(pt);
			avg = (X + (avg * j)) / (j + 1);

#ifdef VISUALIZE
			(*landm_obj).plotPMap(X, 200);
		  (*landm_obj).plotGrid(tmp, 100);
#endif

			if (i == 0 && j == 0) {
				std::cout << "PREV_LINE: " << m.p_line;
				std::cout << "PREV_POS: " << m.p_pos;
				std::cout << "PREV_PT1: " << m.p_line.p1;
				std::cout << "PREV_PT2: " << m.p_line.p2;
				std::cout << "PREV_INC: "
				          << (m.p_line.p2.y - m.p_line.p1.y) /
				                 (m.p_line.p2.x - m.p_line.p1.x)
				          << std::endl;
				std::cout << "CURR_LINE: " << m.c_line;
				std::cout << "CURR_POS: " << m.c_pos;
				std::cout << "CURR_PT1: " << m.c_line.p1;
				std::cout << "CURR_PT2: " << m.c_line.p2;
				std::cout << "CURR_INC: "
				          << (m.c_line.p2.y - m.c_line.p1.y) /
				                 (m.c_line.p2.x - m.c_line.p1.x)
				          << std::endl;
				std::cout << "PROJ_LINE: " << tmp;
				std::cout << "PROJ_PT1: " << tmp.p1;
				std::cout << "PROJ_PT2: " << tmp.p2;
				std::cout << "PROJ_INC: "
				          << (tmp.p2.y - tmp.p1.y) / (tmp.p2.x - tmp.p1.x) << std::endl;
			}
		}

		if (i == 0)
			std::cout << "INTERCEPTION: " << avg;

		Point<double> p         = prev_pose.pos + avg;
		bool          was_found = false;
		size_t        k;
		for (k = 0; k < landmarks.size(); k++) {
			if ((*landm_obj).matches[i].p_pos.euc_dist(landmarks[k].image_pos) <
			    20.0) {
			/* if ((*landm_obj).matches[i].p_pos == landmarks[k].image_pos) { */
				was_found = true;
				break;
			}
		}
		if (was_found == true) {
			landmarks[k].image_pos = (*landm_obj).matches[i].c_pos;
			landmarks[k].world_pos = (landmarks[k].world_pos + p) / 2;
			landmarks[k].updateUnc(all_res);
		}
		else
			landmarks.push_back(Landmark<double>(
			    landmarks.size(), p, (*landm_obj).matches[i].c_pos, all_res));
	}

	std::cout << std::endl << " ------ " << std::endl << std::endl;
}

void ParticleFilter::updateWeights(const Pose<double>& delta_pose)
{
	double weight_sum = 0.0;

	for (size_t i = 0; i < particles.size(); i++) {
		Particle<double> p = particles[i];

		double weight =
		    exp(-0.5 * pow(delta_pose.pos.x - p.pos.x, 2) / pow(STD_X, 2)) *
		    exp(-0.5 * pow(delta_pose.pos.y - p.pos.y, 2) / pow(STD_Y, 2)) *
		    exp(-0.5 * pow(delta_pose.theta - p.theta, 2) / pow(STD_THETA, 2));

		weight_sum += weight;
		particles[i].weight = weight;
	}

	for (size_t i = 0; i < particles.size(); i++) {
		particles[i].weight /= weight_sum;
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample()
{
	std::default_random_engine gen;

	std::discrete_distribution<int> distribution(weights.begin(), weights.end());
	std::vector<Particle<double>>   resampled_particles;

	for (size_t i = 0; i < particles.size(); i++)
		resampled_particles.push_back(particles[distribution(gen)]);

	particles = resampled_particles;
}
