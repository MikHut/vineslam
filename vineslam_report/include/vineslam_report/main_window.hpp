#pragma once

#include <QtGui/QMainWindow>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "ui_main_window.h"
#include "qnode.hpp"

class MainWindowDesign;
namespace vineslam_report
{

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();

  void closeEvent(QCloseEvent* event); // Overloaded function
  void showNoMasterMessage();

public Q_SLOTS:
  void on_actionAbout_triggered();
  void on_button_connect_clicked(bool check);
  void on_button_pause_clicked(bool check);
  void on_button_play_clicked(bool check);
  void on_button_iterate_clicked(bool check);
  void on_check_box_high_level_clicked(bool check);
  void on_check_box_corners_clicked(bool check);
  void on_check_box_planar_clicked(bool check);
  void on_check_box_planes_clicked(bool check);
  void on_check_box_ground_clicked(bool check);
  void on_check_box_image_features_clicked(bool check);
  void on_check_box_gps_clicked(bool check);
  void on_draw_hists_triggered(const cv::Mat& bx_hist,
                               const cv::Mat& by_hist,
                               const cv::Mat& bz_hist,
                               const cv::Mat& ax_hist,
                               const cv::Mat& ay_hist,
                               const cv::Mat& az_hist,
                               const cv::Mat& bR_hist,
                               const cv::Mat& bP_hist,
                               const cv::Mat& bY_hist,
                               const cv::Mat& aR_hist,
                               const cv::Mat& aP_hist,
                               const cv::Mat& aY_hist);

  void updateLoggingView(); // no idea why this can't connect automatically

private:
  Ui::MainWindowDesign ui;
  QNode                qnode;
};

} // namespace vineslam_report
