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

  void updateLoggingView(); // no idea why this can't connect automatically

private:
  Ui::MainWindowDesign ui;
  QNode                qnode;
};

} // namespace vineslam_report
