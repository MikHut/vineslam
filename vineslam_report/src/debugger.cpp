#include "../include/vineslam_report/debugger.hpp"

void Debugger::setReport(const vineslam_msgs::report& report) { m_report = report; }

cv::Mat Debugger::plotXYZHists()
{
  int     rows          = 255;
  int     cols          = 1080;
  int     bar_width     = 3;
  float   spatial_width = 1.0;
  float   spatial_res   = 0.1;
  float   n             = static_cast<float>(cols / bar_width);
  int     hist_size     = static_cast<int>(cols / bar_width);
  cv::Mat hist          = cv::Mat::zeros(rows + 1, cols + 1, CV_8UC1);

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
                                 ((cols / bar_width) / (2 * spatial_width)));
    int idx_y = static_cast<int>((particle.pose.position.y - bhist_ymin) *
                                 ((cols / bar_width) / (2 * spatial_width)));
    int idx_z = static_cast<int>((particle.pose.position.z - bhist_zmin) *
                                 ((cols / bar_width) / (2 * spatial_width)));

    if (idx_x >= n || idx_y >= n || idx_z >= n) {
      ROS_WARN("histXYZ(): out of bounds.");
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

    if (idx_x >= n || idx_y >= n || idx_z >= n) {
      ROS_WARN("histXYZ(): out of bounds.");
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
    int i = bhist_xvec[idx] * cols / bsize;
    //    std::cout << bhist_xvec[idx] << " - > " << i << std::endl;
    cv::rectangle(hist,
                  cv::Point(idx * bar_width, cols),
                  cv::Point(idx * bar_width + bar_width, cols - i),
                  20);
  }
  for (size_t idx = 0; idx < bhist_yvec.size(); idx++) {
    int i = bhist_yvec[idx] * cols / bsize;
    //    std::cout << bhist_yvec[idx] << " - > " << i << std::endl;
    cv::rectangle(hist,
                  cv::Point(idx * bar_width, cols),
                  cv::Point(idx * bar_width + bar_width, cols - i),
                  20);
  }
  for (size_t idx = 0; idx < bhist_zvec.size(); idx++) {
    int i = bhist_zvec[idx] * cols / bsize;
    //    std::cout << bhist_zvec[idx] << " - > " << i << std::endl;
    cv::rectangle(hist,
                  cv::Point(idx * bar_width, cols),
                  cv::Point(idx * bar_width + bar_width, cols - i),
                  20);
  }
  for (size_t idx = 0; idx < ahist_xvec.size(); idx++) {
    int i = ahist_xvec[idx] * cols / asize;
    //    std::cout << ahist_xvec[idx] << " - > " << i << std::endl;
    cv::rectangle(hist,
                  cv::Point(idx * bar_width, cols),
                  cv::Point(idx * bar_width + bar_width, cols - i),
                  20);
  }
  for (size_t idx = 0; idx < ahist_yvec.size(); idx++) {
    int i = ahist_yvec[idx] * cols / asize;
    //    std::cout << ahist_yvec[idx] << " - > " << i << std::endl;
    cv::rectangle(hist,
                  cv::Point(idx * bar_width, cols),
                  cv::Point(idx * bar_width + bar_width, cols - i),
                  20);
  }
  for (size_t idx = 0; idx < ahist_zvec.size(); idx++) {
    int i = ahist_zvec[idx] * cols / asize;
    //    std::cout << ahist_zvec[idx] << " - > " << i << std::endl;
    cv::rectangle(hist,
                  cv::Point(idx * bar_width, cols),
                  cv::Point(idx * bar_width + bar_width, cols - i),
                  20);
  }

  cv::Mat cm_hist;
  cv::applyColorMap(hist, cm_hist, cv::COLORMAP_JET);
  cv::imwrite("/home/andreaguiar/im.png", cm_hist);

  QImage qhist = QImage((uchar*) hist.data, hist.cols, hist.rows, hist.step, QImage::Format_RGB888);

  return hist;
}

cv::Mat Debugger::plotRPYHists() {}