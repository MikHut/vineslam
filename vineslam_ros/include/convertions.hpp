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
struct Convertions
{
  static void pose2TransformStamped(const tf2::Quaternion& q, const tf2::Vector3& t,
                                    geometry_msgs::TransformStamped& tf)
  {
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
  }

  // static geometry_msgs::Point point2GeometryMsgsPoint(const Point& vineslam_point)
  //{
  //  geometry_msgs::Point ros_point;
  //  ros_point.x = vineslam_point.x_;
  //  ros_point.y = vineslam_point.y_;
  //  ros_point.z = vineslam_point.z_;
  //
  //  return ros_point;
  //}

  static void GNSS2UTM(const double& latitute, const double& longitude, double& utm_north, double& utm_east,
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

    double a = WGS84_A;
    double ecc_squared = UTM_E2;
    double k0 = UTM_K0;

    double longiture_origin;
    double ecc_prime_squared;
    double N, T, C, A, M;

    // Make sure the longitude is between -180.00 .. 179.9
    double long_tmp = (longitude + 180.0f) - static_cast<int>((longitude + 180.0f) / 360.0f) * 360.0f - 180.0f;

    double lat_rad = latitute * DEGREE_TO_RAD;
    double long_rad = long_tmp * DEGREE_TO_RAD;
    double long_origin_rad;

    int zone_number = static_cast<int>((long_tmp + 180.0f) / 6.0f) + 1.0f;

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
    longiture_origin = (zone_number - 1.0f) * 6.0f - 180.0f + 3.0f;
    long_origin_rad = longiture_origin * DEGREE_TO_RAD;

    // Compute the UTM Zone from the latitude and longitude
    char zone_buf[] = { 0, 0, 0, 0 };
    snprintf(zone_buf, sizeof(zone_buf), "%hu%c", zone_number, UTMLetterDesignator(latitute));
    utm_zone = std::string(zone_buf);

    ecc_prime_squared = (ecc_squared) / (1.0f - ecc_squared);

    N = a / sqrt(1.0f - ecc_squared * sin(lat_rad) * sin(lat_rad));
    T = tan(lat_rad) * tan(lat_rad);
    C = ecc_prime_squared * cos(lat_rad) * cos(lat_rad);
    A = cos(lat_rad) * (long_rad - long_origin_rad);

    M = a * ((1.0f - ecc_squared / 4.0f - 3.0f * ecc_squared * ecc_squared / 64.0f -
              5.0f * ecc_squared * ecc_squared * ecc_squared / 256.0f) *
                 lat_rad -
             (3.0f * ecc_squared / 8.0f + 3.0f * ecc_squared * ecc_squared / 32.0f +
              45.0f * ecc_squared * ecc_squared * ecc_squared / 1024.0f) *
                 sin(2.0f * lat_rad) +
             (15.0f * ecc_squared * ecc_squared / 256.0f + 45.0f * ecc_squared * ecc_squared * ecc_squared / 1024.0f) *
                 sin(4.0f * lat_rad) -
             (35.0f * ecc_squared * ecc_squared * ecc_squared / 3072.0f) * sin(6.0f * lat_rad));

    utm_east = static_cast<double>(
        k0 * N *
            (A + (1.0f - T + C) * A * A * A / 6.0f +
             (5.0f - 18.0f * T + T * T + 72.0f * C - 58.0f * ecc_prime_squared) * A * A * A * A * A / 120.0f) +
        500000.0);

    utm_north =
        static_cast<double>(k0 * (M + N * tan(lat_rad) *
                                          (A * A / 2.0f + (5.0f - T + 9.0f * C + 4.0f * C * C) * A * A * A * A / 24.0f +
                                           (61.0f - 58.0f * T + T * T + 600.0f * C - 330.0f * ecc_prime_squared) * A *
                                               A * A * A * A * A / 720.0f)));

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
    ecc_primed_squared = (ecc_squared) / (1.0f - ecc_squared);

    M = y / k0;
    mu = M / (a * (1.0f - ecc_squared / 4.0f - 3.0f * ecc_squared * ecc_squared / 64.0f -
                   5.0f * ecc_squared * ecc_squared * ecc_squared / 256.0f));

    phi1_rad = mu + ((3.0f * e1 / 2.0f - 27.0f * e1 * e1 * e1 / 32.0f) * sin(2.0f * mu) +
                     (21.0f * e1 * e1 / 16.0f - 55.0f * e1 * e1 * e1 * e1 / 32.0f) * sin(4.0f * mu) +
                     (151.0f * e1 * e1 * e1 / 96.0f) * sin(6.0f * mu));

    N1 = a / sqrt(1.0f - ecc_squared * sin(phi1_rad) * sin(phi1_rad));
    T1 = tan(phi1_rad) * tan(phi1_rad);
    C1 = ecc_primed_squared * cos(phi1_rad) * cos(phi1_rad);
    R1 = a * (1.0f - ecc_squared) / pow(1.0f - ecc_squared * sin(phi1_rad) * sin(phi1_rad), 1.5);
    D = x / (N1 * k0);

    latitude = phi1_rad -
               ((N1 * tan(phi1_rad) / R1) *
                (D * D / 2.0f -
                 (5.0f + 3.0f * T1 + 10.0f * C1 - 4.0f * C1 * C1 - 9.0f * ecc_primed_squared) * D * D * D * D / 24.0f +
                 (61.0f + 90.0f * T1 + 298.0f * C1 + 45.0f * T1 * T1 - 252.0f * ecc_primed_squared - 3.0f * C1 * C1) *
                     D * D * D * D * D * D / 720.0f));

    latitude = latitude * RAD_TO_DEGREE;

    longitude = ((D - (1.0f + 2.0f * T1 + C1) * D * D * D / 6.0f +
                  (5.0f - 2.0f * C1 + 28.0f * T1 - 3.0f * C1 * C1 + 8.0f * ecc_primed_squared + 24.0f * T1 * T1) * D *
                      D * D * D * D / 120.0f) /
                 cos(phi1_rad));
    longitude = long_origin + longitude * RAD_TO_DEGREE;
  }
};

}  // namespace vineslam