#pragma once

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace wildSLAM
{
class ellipse2D
{
public:
  // Default constructor
  ellipse2D()
  {
    stdX = 0.;
    stdY = 0.;
    TH   = 0.;
  }

  // Construct with values
  ellipse2D(const float& stdX_, const float& stdY_, const float& TH_)
  {
    stdX = stdX_;
    stdY = stdY_;
    TH   = TH_;
  }

  // Construct with another ellipse2D
  ellipse2D(const ellipse2D& other)
  {
    stdX = other.stdX;
    stdY = other.stdY;
    TH   = other.TH;
  }

  // Assignment operator
  ellipse2D operator=(const ellipse2D& other)
  {
    stdX = other.stdX;
    stdY = other.stdY;
    TH   = other.TH;

    return *this;
  }

  // Convert ellipse axis to a Eigen VectorXd
  Eigen::VectorXd toEig()
  {
    Eigen::VectorXd vec(2, 1);
    vec << stdX, stdY;
    return vec;
  }

  // Gaussian standard deviation and ellipse orientation
  float stdX;
  float stdY;
  float TH;

private:
};
}; // namespace wildSLAM
