#pragma once

#include <iostream>
#include <vector>

#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/const.hpp>
#include <vineslam/extern/thread_pool.h>

namespace vineslam
{
static void estimateNormal(const std::vector<Point>& points, float& a, float& b, float& c, float& d)
{
  // -------------------------------------------------------------------------------
  // ----- Use PCA to refine th normal vector using all the inliers
  // -------------------------------------------------------------------------------
  // - 1st: assemble data matrix
  Eigen::MatrixXf data_mat;
  data_mat.conservativeResize(points.size(), 3);
  for (size_t i = 0; i < points.size(); i++)
  {
    Point pt = points[i];
    Eigen::Matrix<float, 1, 3> pt_mat(pt.x_, pt.y_, pt.z_);
    data_mat.block<1, 3>(i, 0) = pt_mat;
  }
  // - 2nd: calculate mean and subtract it to the data matrix
  Eigen::MatrixXf centered_mat = data_mat.rowwise() - data_mat.colwise().mean();
  // - 3rd: calculate covariance matrix
  Eigen::MatrixXf covariance_mat = (centered_mat.adjoint() * centered_mat);
  // - 4rd: calculate eigenvectors and eigenvalues of the covariance matrix
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigen_solver(covariance_mat);
  const Eigen::VectorXf& eigen_values = eigen_solver.eigenvalues();
  const Eigen::MatrixXf& eigen_vectors = eigen_solver.eigenvectors();

  Vec normal = Vec(eigen_vectors.col(0)[0], eigen_vectors.col(0)[1], eigen_vectors.col(0)[2]);

  // -------------------------------------------------------------------------------
  // ----- Normalize and save normal vector
  // -------------------------------------------------------------------------------
  normal.normalize();

  Point avg_pt(0, 0, 0);
  for (const auto& pt : points)
  {
    avg_pt.x_ += pt.x_;
    avg_pt.y_ += pt.y_;
    avg_pt.z_ += pt.z_;
  }
  avg_pt.x_ = (points.empty()) ? 0 : avg_pt.x_ / static_cast<float>(points.size());
  avg_pt.y_ = (points.empty()) ? 0 : avg_pt.y_ / static_cast<float>(points.size());
  avg_pt.z_ = (points.empty()) ? 0 : avg_pt.z_ / static_cast<float>(points.size());

  a = normal.x_;
  b = normal.y_;
  c = normal.z_;
  d = -(normal.x_ * avg_pt.x_ + normal.y_ * avg_pt.y_ + normal.z_ * avg_pt.z_);
}

static bool ransac(const std::vector<Point>& in_pts, Plane& out_plane, int max_iters = 20, float dist_threshold = 0.08,
                   bool filter_distant_pts = false)
{
  std::vector<Point> pts;
  if (filter_distant_pts)
  {
    for (const auto& pt : in_pts)
    {
      if (pt.norm3D() < 5)
      {
        pts.push_back(pt);
      }
    }
  }
  else
  {
    pts = in_pts;
  }

  if (pts.empty())
  {
    return false;
  }

  int max_idx = static_cast<int>(pts.size()) - 1;
  int min_idx = 0;
  int max_tries = 1000;
  int c_max_inliers = 0;

  for (int i = 0; i < max_iters; i++)
  {
    // Declare private point cloud to store current solution
    std::vector<Point> l_pcl;

    // Reset number of inliers in each iteration
    int num_inliers = 0;

    // Randomly select three points that cannot be cohincident
    // TODO (André Aguiar): Also check if points are collinear
    bool found_valid_pts = false;
    int n = 0;
    int idx1, idx2, idx3;
    while (!found_valid_pts)
    {
      idx1 = std::rand() % (max_idx - min_idx + 1) + min_idx;
      idx2 = std::rand() % (max_idx - min_idx + 1) + min_idx;
      idx3 = std::rand() % (max_idx - min_idx + 1) + min_idx;

      if (idx1 != idx2 && idx1 != idx3 && idx2 != idx3)
        found_valid_pts = true;

      n++;
      if (n > max_tries)
        break;
    }

    if (!found_valid_pts)
    {
      std::cout << "WARNING (ransac): No valid set of points found ... " << std::endl;
      return false;
    }

    // Declare the 3 points selected on this iteration
    Point pt1 = Point(pts[idx1].x_, pts[idx1].y_, pts[idx1].z_);
    Point pt2 = Point(pts[idx2].x_, pts[idx2].y_, pts[idx2].z_);
    Point pt3 = Point(pts[idx3].x_, pts[idx3].y_, pts[idx3].z_);

    // Extract the plane hessian coefficients
    Vec v1(pt2, pt1);
    Vec v2(pt3, pt1);
    Vec abc = v1.cross(v2);
    float l_a = abc.x_;
    float l_b = abc.y_;
    float l_c = abc.z_;
    float l_d = -(l_a * pt1.x_ + l_b * pt1.y_ + l_c * pt1.z_);

    for (const auto& l_pt : pts)
    {
      // Compute the distance each point to the plane - from
      // https://www.geeksforgeeks.org/distance-between-a-point-and-a-plane-in-3-d/
      auto norm = std::sqrt(l_a * l_a + l_b * l_b + l_c * l_c);
      if (std::fabs(l_a * l_pt.x_ + l_b * l_pt.y_ + l_c * l_pt.z_ + l_d) / norm < dist_threshold)
      {
        num_inliers++;
        l_pcl.push_back(l_pt);
      }
    }

    if (num_inliers > c_max_inliers)
    {
      c_max_inliers = num_inliers;

      out_plane.points_.clear();
      out_plane.points_ = l_pcl;
      out_plane.a_ = l_a;
      out_plane.b_ = l_b;
      out_plane.c_ = l_c;
      out_plane.d_ = l_d;
    }
  }

  // PCA-based normal refinement using all the inliers
  float l_a, l_b, l_c, l_d;
  estimateNormal(out_plane.points_, l_a, l_b, l_c, l_d);

  out_plane.a_ = l_a;
  out_plane.b_ = l_b;
  out_plane.c_ = l_c;
  out_plane.d_ = l_d;

  return c_max_inliers > 0;
}

static bool ransac(const std::vector<Point>& in_pts, SemiPlane& out_plane, int max_iters = 20,
                   float dist_threshold = 0.08)
{
  Plane l_plane;
  bool success = ransac(in_pts, l_plane, max_iters, dist_threshold);
  out_plane = SemiPlane(l_plane, {});

  return success;
}

}  // namespace vineslam