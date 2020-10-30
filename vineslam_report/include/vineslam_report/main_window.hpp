#pragma once

#include <QtGui/QMainWindow>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "ui_main_window.h"
#include "qnode.hpp"

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
  void on_draw_hists_triggered(cv::Mat bx_hist,
                               cv::Mat by_hist,
                               cv::Mat bz_hist,
                               cv::Mat ax_hist,
                               cv::Mat ay_hist,
                               cv::Mat az_hist,
                               cv::Mat bR_hist,
                               cv::Mat bP_hist,
                               cv::Mat bY_hist,
                               cv::Mat aR_hist,
                               cv::Mat aP_hist,
                               cv::Mat aY_hist);

  void updateLoggingView(); // no idea why this can't connect automatically

private:
  Ui::MainWindowDesign ui;
  QNode                qnode;
};

} // namespace vineslam_report
