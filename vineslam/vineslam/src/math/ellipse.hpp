#pragma once

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace vineslam
{
struct ellipse {
  // Default constructor
  ellipse()
  {
    stdX = 0.;
    stdY = 0.;
    TH   = 0.;
  }

  // Construct with values
  ellipse(const float& stdX_, const float& stdY_, const float& TH_)
  {
    stdX = stdX_;
    stdY = stdY_;
    TH   = TH_;
  }

  // Construct with another ellipse
  ellipse(const ellipse& other)
  {
    stdX = other.stdX;
    stdY = other.stdY;
    TH   = other.TH;
  }

  // Assignment operator
  ellipse operator=(const ellipse& other)
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
};
}; // namespace vineslam
