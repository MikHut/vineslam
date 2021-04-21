#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// WGS84 Parameters
#define WGS84_A 6378137.0     // major axis
#define WGS84_E 0.0818191908  // first eccentricity

// UTM Parameters
#define UTM_K0 0.9996               // scale factor
#define UTM_E2 (WGS84_E * WGS84_E)  // e^2

namespace vineslam
{
static void pose2TransformStamped(const tf2::Quaternion& q, const tf2::Vector3& t,
                                  geometry_msgs::msg::TransformStamped& tf)
{
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();
}

//static geometry_msgs::msg::Point point2GeometryMsgsPoint(const Point& vineslam_point)
//{
//  geometry_msgs::msg::Point ros_point;
//  ros_point.x = vineslam_point.x_;
//  ros_point.y = vineslam_point.y_;
//  ros_point.z = vineslam_point.z_;
//
//  return ros_point;
//}

static void GNSS2UTM(const float& latitute, const float& longitude, float& utm_north, float& utm_east,
                     std::string& utm_zone)
{
  auto UTMLetterDesignator = [](float latitude) {
    if ((84 >= latitude) && (latitude >= 72))
      return 'X';
    else if ((72 > latitude) && (latitude >= 64))
      return 'W';
    else if ((64 > latitude) && (latitude >= 56))
      return 'V';
    else if ((56 > latitude) && (latitude >= 48))
      return 'U';
    else if ((48 > latitude) && (latitude >= 40))
      return 'T';
    else if ((40 > latitude) && (latitude >= 32))
      return 'S';
    else if ((32 > latitude) && (latitude >= 24))
      return 'R';
    else if ((24 > latitude) && (latitude >= 16))
      return 'Q';
    else if ((16 > latitude) && (latitude >= 8))
      return 'P';
    else if ((8 > latitude) && (latitude >= 0))
      return 'N';
    else if ((0 > latitude) && (latitude >= -8))
      return 'M';
    else if ((-8 > latitude) && (latitude >= -16))
      return 'L';
    else if ((-16 > latitude) && (latitude >= -24))
      return 'K';
    else if ((-24 > latitude) && (latitude >= -32))
      return 'J';
    else if ((-32 > latitude) && (latitude >= -40))
      return 'H';
    else if ((-40 > latitude) && (latitude >= -48))
      return 'G';
    else if ((-48 > latitude) && (latitude >= -56))
      return 'F';
    else if ((-56 > latitude) && (latitude >= -64))
      return 'E';
    else if ((-64 > latitude) && (latitude >= -72))
      return 'D';
    else if ((-72 > latitude) && (latitude >= -80))
      return 'C';
    else
      return 'Z';
  };

  float a = WGS84_A;
  float ecc_squared = UTM_E2;
  float k0 = UTM_K0;

  float longiture_origin;
  float ecc_prime_squared;
  float N, T, C, A, M;

  // Make sure the longitude is between -180.00 .. 179.9
  float long_tmp = (longitude + 180) - static_cast<int>((longitude + 180) / 360) * 360 - 180;

  float lat_rad = latitute * DEGREE_TO_RAD;
  float long_rad = long_tmp * DEGREE_TO_RAD;
  float long_origin_rad;

  int zone_number = static_cast<int>((long_tmp + 180) / 6) + 1;

  if (latitute >= 56.0 && latitute < 64.0 && long_tmp >= 3.0 && long_tmp < 12.0)
    zone_number = 32;

  // Special zones for Svalbard
  if (latitute >= 72.0 && latitute < 84.0)
  {
    if (long_tmp >= 0.0 && long_tmp < 9.0)
      zone_number = 31;
    else if (long_tmp >= 9.0 && long_tmp < 21.0)
      zone_number = 33;
    else if (long_tmp >= 21.0 && long_tmp < 33.0)
      zone_number = 35;
    else if (long_tmp >= 33.0 && long_tmp < 42.0)
      zone_number = 37;
  }
  // +3 puts origin in middle of zone
  longiture_origin = (zone_number - 1) * 6 - 180 + 3;
  long_origin_rad = longiture_origin * DEGREE_TO_RAD;

  // Compute the UTM Zone from the latitude and longitude
  char zone_buf[] = { 0, 0, 0, 0 };
  snprintf(zone_buf, sizeof(zone_buf), "%hu%c", zone_number, UTMLetterDesignator(latitute));
  utm_zone = std::string(zone_buf);

  ecc_prime_squared = (ecc_squared) / (1 - ecc_squared);

  N = a / sqrt(1 - ecc_squared * sin(lat_rad) * sin(lat_rad));
  T = tan(lat_rad) * tan(lat_rad);
  C = ecc_prime_squared * cos(lat_rad) * cos(lat_rad);
  A = cos(lat_rad) * (long_rad - long_origin_rad);

  M = a *
      ((1 - ecc_squared / 4 - 3 * ecc_squared * ecc_squared / 64 - 5 * ecc_squared * ecc_squared * ecc_squared / 256) *
           lat_rad -
       (3 * ecc_squared / 8 + 3 * ecc_squared * ecc_squared / 32 +
        45 * ecc_squared * ecc_squared * ecc_squared / 1024) *
           sin(2 * lat_rad) +
       (15 * ecc_squared * ecc_squared / 256 + 45 * ecc_squared * ecc_squared * ecc_squared / 1024) * sin(4 * lat_rad) -
       (35 * ecc_squared * ecc_squared * ecc_squared / 3072) * sin(6 * lat_rad));

  utm_east = static_cast<float>(k0 * N *
                                    (A + (1 - T + C) * A * A * A / 6 +
                                     (5 - 18 * T + T * T + 72 * C - 58 * ecc_prime_squared) * A * A * A * A * A / 120) +
                                500000.0);

  utm_north = static_cast<float>(
      k0 * (M + N * tan(lat_rad) *
                    (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                     (61 - 58 * T + T * T + 600 * C - 330 * ecc_prime_squared) * A * A * A * A * A * A / 720)));

  if (latitute < 0)
  {
    // 10000000 meter offset for southern hemisphere
    utm_north += 10000000.0;
  }
}
static void UTMtoGNSS(const float& utm_north, const float& utm_east, const std::string& utm_zone, float& latitude,
                      float& longitude)
{
  double k0 = UTM_K0;
  double a = WGS84_A;
  double ecc_squared = UTM_E2;
  double ecc_primed_squared;
  double e1 = (1 - sqrt(1 - ecc_squared)) / (1 + sqrt(1 - ecc_squared));
  double N1, T1, C1, R1, D, M;
  double long_origin;
  double mu, phi1_rad;
  double x, y;
  int zone_number;
  char* zone_letter;

  x = utm_east - 500000.0;  // remove 500,000 meter offset for longitude
  y = utm_north;

  zone_number = strtoul(utm_zone.c_str(), &zone_letter, 10);
  if ((*zone_letter - 'N') < 0)
  {
    // remove 10,000,000 meter offset used for southern hemisphere
    y -= 10000000.0;
  }

  // +3 puts origin in middle of zone
  long_origin = (zone_number - 1) * 6 - 180 + 3;
  ecc_primed_squared = (ecc_squared) / (1 - ecc_squared);

  M = y / k0;
  mu = M / (a * (1 - ecc_squared / 4 - 3 * ecc_squared * ecc_squared / 64 -
                 5 * ecc_squared * ecc_squared * ecc_squared / 256));

  phi1_rad =
      mu + ((3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu) +
            (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu) + (151 * e1 * e1 * e1 / 96) * sin(6 * mu));

  N1 = a / sqrt(1 - ecc_squared * sin(phi1_rad) * sin(phi1_rad));
  T1 = tan(phi1_rad) * tan(phi1_rad);
  C1 = ecc_primed_squared * cos(phi1_rad) * cos(phi1_rad);
  R1 = a * (1 - ecc_squared) / pow(1 - ecc_squared * sin(phi1_rad) * sin(phi1_rad), 1.5);
  D = x / (N1 * k0);

  latitude =
      phi1_rad - ((N1 * tan(phi1_rad) / R1) *
                  (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * ecc_primed_squared) * D * D * D * D / 24 +
                   (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * ecc_primed_squared - 3 * C1 * C1) * D * D * D * D *
                       D * D / 720));

  latitude = latitude * RAD_TO_DEGREE;

  longitude =
      ((D - (1 + 2 * T1 + C1) * D * D * D / 6 +
        (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * ecc_primed_squared + 24 * T1 * T1) * D * D * D * D * D / 120) /
       cos(phi1_rad));
  longitude = long_origin + longitude * RAD_TO_DEGREE;
}

}  // namespace vineslam