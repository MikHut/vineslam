#include "../include/vineslam_report/main_window.hpp"

namespace vineslam_report
{

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget* parent)
    : QMainWindow(parent)
    , qnode(argc, argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to
                    // on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt,
                   SIGNAL(triggered(bool)),
                   qApp,
                   SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(
      0); // ensure the first tab is showing - qt-designer should have this already
          // hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  // Logging Logging
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(
      &qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void MainWindow::on_button_connect_clicked(bool check)
{
  if (!qnode.init()) {
    showNoMasterMessage();
  } else {
    ui.button_connect->setEnabled(false);
  }
}

void MainWindow::updateLoggingView() { ui.view_logging->scrollToBottom(); }

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this,
                     tr("About ..."),
                     tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin "
                        "Robot</p><p>This package needs an about description.</p>"));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

} // namespace vineslam_report
