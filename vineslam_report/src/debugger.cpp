#include "../include/vineslam_report/debugger.hpp"

void Debugger::setReport(const vineslam_msgs::report& report) { m_report = report; }

void Debugger::plotXYZHists(cv::Mat& bx_hist,
                            cv::Mat& by_hist,
                            cv::Mat& bz_hist,
                            cv::Mat& ax_hist,
                            cv::Mat& ay_hist,
                            cv::Mat& az_hist)
{
  int   rows          = 255;
  int   cols          = 1080;
  int   bar_width     = 2;
  float spatial_width = 0.2;
  float n             = static_cast<float>(cols / bar_width);
  int   hist_size     = static_cast<int>(cols / bar_width);
  bx_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  by_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  bz_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  ax_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  ay_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  az_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);

  // -------------------------------------------------------------------------------
  // ----- Compute average of XYZ components
  // -------------------------------------------------------------------------------
  float bsize   = static_cast<float>(m_report.b_particles.size());
  float bx_mean = 0, by_mean = 0, bz_mean = 0;
  for (const auto& particle : m_report.b_particles) {
    bx_mean += particle.pose.position.x;
    by_mean += particle.pose.position.y;
    bz_mean += particle.pose.position.z;
  }
  bx_mean /= bsize;
  by_mean /= bsize;
  bz_mean /= bsize;

  float asize   = static_cast<float>(m_report.b_particles.size());
  float ax_mean = 0, ay_mean = 0, az_mean = 0;
  for (const auto& particle : m_report.a_particles) {
    ax_mean += particle.pose.position.x;
    ay_mean += particle.pose.position.y;
    az_mean += particle.pose.position.z;
  }
  ax_mean /= asize;
  ay_mean /= asize;
  az_mean /= asize;

  // -------------------------------------------------------------------------------
  // ----- Compute histogram limits
  // -------------------------------------------------------------------------------
  float bhist_xmin = bx_mean - spatial_width, bhist_xmax = bx_mean + spatial_width;
  float bhist_ymin = by_mean - spatial_width, bhist_ymax = by_mean + spatial_width;
  float bhist_zmin = bz_mean - spatial_width, bhist_zmax = bz_mean + spatial_width;
  float ahist_xmin = ax_mean - spatial_width, ahist_xmax = ax_mean + spatial_width;
  float ahist_ymin = ay_mean - spatial_width, ahist_ymax = ay_mean + spatial_width;
  float ahist_zmin = az_mean - spatial_width, ahist_zmax = az_mean + spatial_width;

  // -------------------------------------------------------------------------------
  // ----- Compute XYZ histograms
  // -------------------------------------------------------------------------------
  std::vector<int> bhist_xvec(hist_size, 0);
  std::vector<int> bhist_yvec(hist_size, 0);
  std::vector<int> bhist_zvec(hist_size, 0);
  for (const auto& particle : m_report.b_particles) {
    int idx_x = static_cast<int>((particle.pose.position.x - bhist_xmin) *
                                 (n / (2 * spatial_width)));
    int idx_y = static_cast<int>((particle.pose.position.y - bhist_ymin) *
                                 (n / (2 * spatial_width)));
    int idx_z = static_cast<int>((particle.pose.position.z - bhist_zmin) *
                                 (n / (2 * spatial_width)));

    if (idx_x < 0 || idx_x >= n || idx_y < 0 || idx_y >= n || idx_z < 0 ||
        idx_z >= n) {
      continue;
    }

    bhist_xvec[idx_x]++;
    bhist_yvec[idx_y]++;
    bhist_zvec[idx_z]++;
  }
  std::vector<int> ahist_xvec(hist_size, 0);
  std::vector<int> ahist_yvec(hist_size, 0);
  std::vector<int> ahist_zvec(hist_size, 0);
  for (const auto& particle : m_report.a_particles) {
    int idx_x = static_cast<int>((particle.pose.position.x - ahist_xmin) *
                                 (n / (2 * spatial_width)));
    int idx_y = static_cast<int>((particle.pose.position.y - ahist_ymin) *
                                 (n / (2 * spatial_width)));
    int idx_z = static_cast<int>((particle.pose.position.z - ahist_zmin) *
                                 (n / (2 * spatial_width)));

    if (idx_x < 0 || idx_x >= n || idx_y < 0 || idx_y >= n || idx_z < 0 ||
        idx_z >= n) {
      continue;
    }

    ahist_xvec[idx_x]++;
    ahist_yvec[idx_y]++;
    ahist_zvec[idx_z]++;
  }

  // -------------------------------------------------------------------------------
  // ----- Draw histograms on cv image
  // -------------------------------------------------------------------------------
  for (size_t idx = 0; idx < bhist_xvec.size(); idx++) {
    int i = bhist_xvec[idx] * rows / bsize * 5;
    cv::rectangle(bx_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < bhist_yvec.size(); idx++) {
    int i = bhist_yvec[idx] * rows / bsize * 5;
    cv::rectangle(by_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < bhist_zvec.size(); idx++) {
    int i = bhist_zvec[idx] * rows / bsize * 5;
    cv::rectangle(bz_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < ahist_xvec.size(); idx++) {
    int i = ahist_xvec[idx] * rows / asize * 5;
    cv::rectangle(ax_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < ahist_yvec.size(); idx++) {
    int i = ahist_yvec[idx] * rows / asize * 5;
    cv::rectangle(ay_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < ahist_zvec.size(); idx++) {
    int i = ahist_zvec[idx] * rows / asize * 5;
    cv::rectangle(az_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }

  // -------------------------------------------------------------------------------
  // ----- Draw histograms axes
  // -------------------------------------------------------------------------------
  cv::arrowedLine(bx_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(by_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(bz_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(ax_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(ay_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(az_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);

  cv::putText(bx_hist,
              to_string_with_precision(bhist_xmin),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bx_hist,
              to_string_with_precision(bx_mean),
              cv::Point(cols / 2 - 35, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bx_hist,
              to_string_with_precision(bhist_xmax),
              cv::Point(cols - 100, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(by_hist,
              to_string_with_precision(bhist_ymin),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(by_hist,
              to_string_with_precision(by_mean),
              cv::Point(cols / 2 - 35, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(by_hist,
              to_string_with_precision(bhist_ymax),
              cv::Point(cols - 100, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bz_hist,
              to_string_with_precision(bhist_zmin),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bz_hist,
              to_string_with_precision(bz_mean),
              cv::Point(cols / 2 - 35, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bz_hist,
              to_string_with_precision(bhist_zmax),
              cv::Point(cols - 100, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(ax_hist,
              to_string_with_precision(ahist_xmin),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(ax_hist,
              to_string_with_precision(ax_mean),
              cv::Point(cols / 2 - 35, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(ax_hist,
              to_string_with_precision(ahist_xmax),
              cv::Point(cols - 100, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(ay_hist,
              to_string_with_precision(ahist_ymin),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(ay_hist,
              to_string_with_precision(ay_mean),
              cv::Point(cols / 2 - 35, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(ay_hist,
              to_string_with_precision(ahist_ymax),
              cv::Point(cols - 100, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(az_hist,
              to_string_with_precision(ahist_zmin),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(az_hist,
              to_string_with_precision(az_mean),
              cv::Point(cols / 2 - 35, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(az_hist,
              to_string_with_precision(ahist_zmax),
              cv::Point(cols - 100, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
}

void Debugger::plotRPYHists(cv::Mat& bR_hist,
                            cv::Mat& bP_hist,
                            cv::Mat& bY_hist,
                            cv::Mat& aR_hist,
                            cv::Mat& aP_hist,
                            cv::Mat& aY_hist)
{
  int   rows          = 255;
  int   cols          = 1080;
  int   bar_width     = 2;
  float angular_width = M_PI;
  float n             = static_cast<float>(cols / bar_width);
  int   hist_size     = static_cast<int>(cols / bar_width);
  bR_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  bP_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  bY_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  aR_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  aP_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);
  aY_hist             = cv::Mat::zeros(rows + 50, cols + 1, CV_8UC3);

  // -------------------------------------------------------------------------------
  // ----- Compute histogram limits
  // -------------------------------------------------------------------------------
  float min = -M_PI, max = M_PI;

  // -------------------------------------------------------------------------------
  // ----- Compute XYZ histograms
  // -------------------------------------------------------------------------------
  float bsize = static_cast<float>(m_report.b_particles.size());
  float asize = static_cast<float>(m_report.a_particles.size());

  std::vector<int> bhist_Rvec(hist_size, 0);
  std::vector<int> bhist_Pvec(hist_size, 0);
  std::vector<int> bhist_Yvec(hist_size, 0);
  for (const auto& particle : m_report.b_particles) {
    tf::Quaternion q(particle.pose.orientation.x,
                     particle.pose.orientation.y,
                     particle.pose.orientation.z,
                     particle.pose.orientation.w);
    tf::Matrix3x3  m(q);
    double         roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    int idx_R = static_cast<int>((roll - min) * (n / (2 * angular_width)));
    int idx_P = static_cast<int>((pitch - min) * (n / (2 * angular_width)));
    int idx_Y = static_cast<int>((yaw - min) * (n / (2 * angular_width)));

    if (idx_R < 0 || idx_R >= n || idx_P < 0 || idx_P >= n || idx_Y < 0 ||
        idx_Y >= n) {
      continue;
    }

    bhist_Rvec[idx_R]++;
    bhist_Pvec[idx_P]++;
    bhist_Yvec[idx_Y]++;
  }
  std::vector<int> ahist_Rvec(hist_size, 0);
  std::vector<int> ahist_Pvec(hist_size, 0);
  std::vector<int> ahist_Yvec(hist_size, 0);
  for (const auto& particle : m_report.a_particles) {
    tf::Quaternion q(particle.pose.orientation.x,
                     particle.pose.orientation.y,
                     particle.pose.orientation.z,
                     particle.pose.orientation.w);
    tf::Matrix3x3  m(q);
    double         roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    int idx_R = static_cast<int>((roll - min) * (n / (2 * angular_width)));
    int idx_P = static_cast<int>((pitch - min) * (n / (2 * angular_width)));
    int idx_Y = static_cast<int>((yaw - min) * (n / (2 * angular_width)));

    if (idx_R < 0 || idx_R >= n || idx_P < 0 || idx_P >= n || idx_Y < 0 ||
        idx_Y >= n) {
      continue;
    }

    ahist_Rvec[idx_R]++;
    ahist_Pvec[idx_P]++;
    ahist_Yvec[idx_Y]++;
  }

  // -------------------------------------------------------------------------------
  // ----- Draw histograms on cv image
  // -------------------------------------------------------------------------------
  for (size_t idx = 0; idx < bhist_Rvec.size(); idx++) {
    int i = bhist_Rvec[idx] * rows / bsize * 5;
    cv::rectangle(bR_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < bhist_Pvec.size(); idx++) {
    int i = bhist_Pvec[idx] * rows / bsize * 5;
    cv::rectangle(bP_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < bhist_Yvec.size(); idx++) {
    int i = bhist_Yvec[idx] * rows / bsize * 5;
    cv::rectangle(bY_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < ahist_Rvec.size(); idx++) {
    int i = ahist_Rvec[idx] * rows / asize * 5;
    cv::rectangle(aR_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < ahist_Pvec.size(); idx++) {
    int i = ahist_Pvec[idx] * rows / asize * 5;
    cv::rectangle(aP_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }
  for (size_t idx = 0; idx < ahist_Yvec.size(); idx++) {
    int i = ahist_Yvec[idx] * rows / asize * 5;
    cv::rectangle(aY_hist,
                  cv::Point(idx * bar_width, rows),
                  cv::Point(idx * bar_width + bar_width, rows - i),
                  cv::Scalar(255, 255, 255),
                  CV_FILLED);
  }

  // -------------------------------------------------------------------------------
  // ----- Draw histograms axes
  // -------------------------------------------------------------------------------
  cv::arrowedLine(bR_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(bP_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(bY_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(aR_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(aP_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);
  cv::arrowedLine(aY_hist,
                  cv::Point(10, rows),
                  cv::Point(cols - 10, rows),
                  cv::Scalar(255, 255, 255),
                  2,
                  8,
                  0,
                  0.01);

  cv::putText(bR_hist,
              to_string_with_precision(min * 180 / M_PI, 0),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bR_hist,
              to_string_with_precision(0),
              cv::Point(cols / 2, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bR_hist,
              to_string_with_precision(max * 180 / M_PI, 0),
              cv::Point(cols - 50, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bP_hist,
              to_string_with_precision(min * 180 / M_PI, 0),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bP_hist,
              to_string_with_precision(0),
              cv::Point(cols / 2, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bP_hist,
              to_string_with_precision(max * 180 / M_PI, 0),
              cv::Point(cols - 50, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bY_hist,
              to_string_with_precision(min * 180 / M_PI, 0),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bY_hist,
              to_string_with_precision(0),
              cv::Point(cols / 2, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(bY_hist,
              to_string_with_precision(max * 180 / M_PI, 0),
              cv::Point(cols - 50, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(aR_hist,
              to_string_with_precision(min * 180 / M_PI, 0),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(aR_hist,
              to_string_with_precision(0),
              cv::Point(cols / 2, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(aR_hist,
              to_string_with_precision(max * 180 / M_PI, 0),
              cv::Point(cols - 50, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(aP_hist,
              to_string_with_precision(min * 180 / M_PI, 0),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(aP_hist,
              to_string_with_precision(0),
              cv::Point(cols / 2, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(aP_hist,
              to_string_with_precision(max * 180 / M_PI, 0),
              cv::Point(cols - 50, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(aY_hist,
              to_string_with_precision(min * 180 / M_PI, 0),
              cv::Point(3, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(aY_hist,
              to_string_with_precision(0),
              cv::Point(cols / 2, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
  cv::putText(aY_hist,
              to_string_with_precision(max * 180 / M_PI, 0),
              cv::Point(cols - 50, rows + 35),
              cv::FONT_HERSHEY_DUPLEX,
              0.7,
              CV_RGB(255, 255, 255),
              1);
}