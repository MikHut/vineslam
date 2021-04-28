#pragma once

#include <iostream>
#include <cmath>

#include <vineslam/math/Tf.hpp>
#include <vineslam/math/Const.hpp>

namespace vineslam
{
// From
// https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_multi_rtk_ros/src/piksi_multi_rtk_ros/piksi_multi.py

struct Geodetic
{
  Geodetic(const double& latitude, const double& longitude, const double& altitude)
  {
    // Set constant variables
    k_semimajor_axis_ = 6378137.0;
    k_first_eccentricity_squared_ = 6.69437999014 * 0.001;

    // Convert inputs to radians
    double lat_rad = latitude * DEGREE_TO_RAD;
    double lon_rad = longitude * DEGREE_TO_RAD;

    // Initialize variables
    geodetic2ecef(latitude, longitude, altitude, initial_ecef_x_, initial_ecef_y_, initial_ecef_z_);

    // Compute ECEF to NED.
    double phi_p = std::atan2(initial_ecef_z_, std::sqrt(std::pow(initial_ecef_x_, 2) + std::pow(initial_ecef_y_, 2)));
    ecef_to_ned_matrix = n_re(phi_p, lon_rad);
  }

  // Geodetic position to local ENU frame
  void geodetic2enu(const double& latitude, const double& longitude, const double& altitude, double& e, double& n,
                    double& u)
  {
    double x, y, z;
    double d;
    geodetic2ecef(latitude, longitude, altitude, x, y, z);
    ecef2ned(x, y, z, n, e, d);
    u = -d;
  }

  // Geodetic position to local NED frame
  void geodetic2ned(const double& latitude, const double& longitude, const double& altitude, double& n, double& e,
                    double& d)
  {
    double x, y, z;
    geodetic2ecef(latitude, longitude, altitude, x, y, z);
    ecef2ned(x, y, z, n, e, d);
  }

  // Convert geodetic coordinates to ECEF.
  void geodetic2ecef(const double& latitude, const double& longitude, const double& altitude, double& x, double& y,
                     double& z)
  {
    double lat_rad = latitude * DEGREE_TO_RAD;
    double lon_rad = longitude * DEGREE_TO_RAD;
    double xi = std::sqrt(1 - k_first_eccentricity_squared_ * std::sin(lat_rad) * std::sin(lat_rad));
    x = (k_semimajor_axis_ / xi + altitude) * std::cos(lat_rad) * std::cos(lon_rad);
    y = (k_semimajor_axis_ / xi + altitude) * std::cos(lat_rad) * std::sin(lon_rad);
    z = (k_semimajor_axis_ / xi * (1 - k_first_eccentricity_squared_) + altitude) * std::sin(lat_rad);
  }

  // Converts ECEF coordinate position into local-tangent-plane NED.
  // Coordinates relative to given ECEF coordinate frame.
  void ecef2ned(const double& x, const double& y, const double& z, double& n, double& e, double& d)
  {
    Point vect(0., 0., 0.);
    vect.x_ = x - initial_ecef_x_;
    vect.y_ = y - initial_ecef_y_;
    vect.z_ = z - initial_ecef_z_;

    Point ret = vect * ecef_to_ned_matrix;
    n = ret.x_;
    e = ret.y_;
    d = -ret.z_;
  }

  Tf n_re(const double& lat_radians, const double& lon_radians)
  {
    double s_lat = std::sin(lat_radians);
    double s_lon = std::sin(lon_radians);
    double c_lat = std::cos(lat_radians);
    double c_lon = std::cos(lon_radians);

    Tf ret = Tf::unitary();

    ret.R_array_[0] = -s_lat * c_lon;
    ret.R_array_[1] = -s_lat * s_lon;
    ret.R_array_[2] = c_lat;
    ret.R_array_[3] = -s_lon;
    ret.R_array_[4] = c_lon;
    ret.R_array_[5] = 0.0;
    ret.R_array_[6] = c_lat * c_lon;
    ret.R_array_[7] = c_lat * s_lon;
    ret.R_array_[8] = s_lat;

    return ret;
  }

  // Initial values
  double initial_ecef_x_;
  double initial_ecef_y_;
  double initial_ecef_z_;
  Tf ecef_to_ned_matrix;

  // Constants
  double k_semimajor_axis_;
  double k_first_eccentricity_squared_;
};
}  // namespace vineslam