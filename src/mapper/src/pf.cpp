#include "../include/mapper/pf.hpp"

PF::PF(const bool& side, const Parameters& params) : params(params)
{
	P = MatrixXd(2, 2);
	P << params.vine_std_x, 0, 0, params.vine_std_y;

	double dy;
	int    i = 0;
	if (side == 0) {
		dy = 0.1;
		while (dy <= 1.5) {
			Particle<double> p(i, dy);
			particles.push_back(p);
			dy += 0.05;
			i++;
		}
	}
	else {
		dy = -1.5;
		while (dy <= -0.1) {
			Particle<double> p(i, dy);
			particles.push_back(p);
			dy += 0.05;
			i++;
		}
	}
}

std::vector<Point<double>> PF::process(const Pose<double>&  c_pose,
                                       const Pose<double>&  p_pose,
                                       const Point<double>& c_col,
                                       const Point<double>& p_col,
                                       const Line<double>&  vine)
{
	std::vector<Point<double>> p = predict(c_pose, p_pose, c_col, p_col, vine);
	if (particles[1].cov < 1e3) {
		update();
		resample();
	}

	return p;
}

std::vector<Point<double>> PF::predict(const Pose<double>&  c_pose,
                                       const Pose<double>&  p_pose,
                                       const Point<double>& c_col,
                                       const Point<double>& p_col,
                                       const Line<double>& vine)
{
	/* extraction of the observation */
	Point<double> X = processObsv(c_col, p_col, c_pose - p_pose);
	VectorXd      z(2, 1);
	z << sqrt((X.x * X.x) + (X.y * X.y)), atan2(X.y, X.x);

	/* distance to the vine line point */
	Line<double>  l_prev  = computeLine(p_col);
	Point<double> vine_pt = l_prev.intercept(vine);

	std::vector<Point<double>> res;
	for (size_t i = 0; i < particles.size(); i++) {
		Point<double> TH = expectEstimation(p_col, p_pose.pos, particles[i].dy);

		/* initialize kalman filter */
		KF kf(TH.eig(), P, params);

		/* kalman filter */
		Pose<double> s = p_pose;
		kf.process(TH.eig(), s.eig(), z);
		VectorXd X = kf.getState();
		MatrixXd R = kf.getObsvCov();

		/* update particle covariance */
		particles[i].cov = R.norm();

		/* update distance to vine hypothetical point */
		particles[i].dist = vine_pt.euc_dist(Point<double>(X[0], X[1]));

		res.push_back(Point<double>(X[0], X[1]));
	}

	return res;
}

void PF::update()
{

	double w_sum = 0;
	/* update particles weights */
	for (size_t i = 0; i < particles.size(); i++) {
		Particle<double> p = particles[i];

		double w = exp(-0.5 * ((p.cov * p.cov)));
		w_sum += w;

		particles[i].w = w;
	}

	/* normalize weights */
	for (size_t i = 0; i < particles.size(); i++)
		particles[i].w /= w_sum;
}

void PF::resample()
{
	const int M = particles.size();

	std::vector<double> w;
	for (int i = 0; i < M; i++)
		w.push_back(particles[i].w);

	std::vector<double> Q(M);
	Q[0] = w[0];
	for (int i = 1; i < M; i++)
		Q[i] = Q[i - 1] + w[i];

	int              i = 0;
	std::vector<int> index(M);
	while (i < M) {
		double sample = ((double)std::rand() / (RAND_MAX));
		int    j      = 1;

		while (Q[j] < sample)
			j++;

		index[i] = j;
		i++;
	}

	for (i = 0; i < M; i++) {
		particles[i].dy  = particles[index[i]].dy;
		particles[i].w   = particles[index[i]].w;
		particles[i].cov = particles[index[i]].cov;
	}
}

Point<double> PF::expectEstimation(const Point<double>& X_prev,
                                   const Point<double>& p_pos, const double& dy)
{
	Line<double> l_prev = computeLine(X_prev, 0);
	Line<double> l_trunk =
	    Line<double>(Point<double>(0, dy), Point<double>(10, dy));

	return (l_prev.intercept(l_trunk) + p_pos);
}

Point<double> PF::processObsv(const Point<double>& c_col,
                              const Point<double>& p_col,
                              const Pose<double>&  delta_p)
{
	Line<double>  l_prev = computeLine(p_col);
	Line<double>  l_proj = projectLine(c_col, delta_p.pos, delta_p.theta);
	Point<double> X      = l_prev.intercept(l_proj);

	return X;
}

/* ******* auxiliar functions ******* */

Line<double> PF::computeLine(const Point<double>& landmark)
{
	double orientation =
	    -(params.h_fov / params.width) * (params.width / 2 - landmark.x);

	Point<double> p1(0, 0);
	Point<double> p2(500 * cos(orientation), 500 * sin(orientation));

	return Line<double>(p1, p2);
}

Line<double> PF::computeLine(const Point<double>& landmark, const double& phi)
{
	double orientation =
	    -(params.h_fov / params.width) * (params.width / 2 - landmark.x);

	Point<double> p1(0, 0);
	Point<double> p2(500 * cos(orientation - phi), 500 * sin(orientation - phi));

	return Line<double>(p1, p2);
}

Line<double> PF::projectLine(const Point<double>& pos,
                             const Point<double>& delta_p,
                             const double&        delta_th)
{
	/* First rotate the line */
	Line<double> l = computeLine(pos, delta_th);
	/* Then, translate the line */
	Point<double> p1 = l.p1 + delta_p;
	Point<double> p2 = l.p2 + delta_p;

	return Line<double>(p1, p2);
}
