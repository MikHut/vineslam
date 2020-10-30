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
  QObject::connect(&qnode,
                   SIGNAL(drawHistOnGui(cv::Mat,
                                        cv::Mat,
                                        cv::Mat,
                                        cv::Mat,
                                        cv::Mat,
                                        cv::Mat,
                                        cv::Mat,
                                        cv::Mat,
                                        cv::Mat,
                                        cv::Mat,
                                        cv::Mat,
                                        cv::Mat)),
                   this,
                   SLOT(on_draw_hists_triggered(cv::Mat,
                                                cv::Mat,
                                                cv::Mat,
                                                cv::Mat,
                                                cv::Mat,
                                                cv::Mat,
                                                cv::Mat,
                                                cv::Mat,
                                                cv::Mat,
                                                cv::Mat,
                                                cv::Mat,
                                                cv::Mat)));
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

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

void MainWindow::on_draw_hists_triggered(cv::Mat bx_hist,
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
                                         cv::Mat aY_hist)
{
  QImage bqx_hist = QImage((uchar*)bx_hist.data,
                           bx_hist.cols,
                           bx_hist.rows,
                           bx_hist.step,
                           QImage::Format_RGB888);
  QImage bqy_hist = QImage((uchar*)by_hist.data,
                           by_hist.cols,
                           by_hist.rows,
                           by_hist.step,
                           QImage::Format_RGB888);
  QImage bqz_hist = QImage((uchar*)bz_hist.data,
                           bz_hist.cols,
                           bz_hist.rows,
                           bz_hist.step,
                           QImage::Format_RGB888);
  QImage aqx_hist = QImage((uchar*)ax_hist.data,
                           ax_hist.cols,
                           ax_hist.rows,
                           ax_hist.step,
                           QImage::Format_RGB888);
  QImage aqy_hist = QImage((uchar*)ay_hist.data,
                           ay_hist.cols,
                           ay_hist.rows,
                           ay_hist.step,
                           QImage::Format_RGB888);
  QImage aqz_hist = QImage((uchar*)az_hist.data,
                           az_hist.cols,
                           az_hist.rows,
                           az_hist.step,
                           QImage::Format_RGB888);
  QImage bqR_hist = QImage((uchar*)bR_hist.data,
                           bR_hist.cols,
                           bR_hist.rows,
                           bR_hist.step,
                           QImage::Format_RGB888);
  QImage bqP_hist = QImage((uchar*)bP_hist.data,
                           bP_hist.cols,
                           bP_hist.rows,
                           bP_hist.step,
                           QImage::Format_RGB888);
  QImage bqY_hist = QImage((uchar*)bY_hist.data,
                           bY_hist.cols,
                           bY_hist.rows,
                           bY_hist.step,
                           QImage::Format_RGB888);
  QImage aqR_hist = QImage((uchar*)aR_hist.data,
                           aR_hist.cols,
                           aR_hist.rows,
                           aR_hist.step,
                           QImage::Format_RGB888);
  QImage aqP_hist = QImage((uchar*)aP_hist.data,
                           aP_hist.cols,
                           aP_hist.rows,
                           aP_hist.step,
                           QImage::Format_RGB888);
  QImage aqY_hist = QImage((uchar*)aY_hist.data,
                           aY_hist.cols,
                           aY_hist.rows,
                           aY_hist.step,
                           QImage::Format_RGB888);

  ui.b_hist_x->setPixmap(QPixmap::fromImage(bqx_hist));
  ui.b_hist_x->setScaledContents(true);
  ui.b_hist_y->setPixmap(QPixmap::fromImage(bqy_hist));
  ui.b_hist_y->setScaledContents(true);
  ui.b_hist_z->setPixmap(QPixmap::fromImage(bqz_hist));
  ui.b_hist_z->setScaledContents(true);
  ui.a_hist_x->setPixmap(QPixmap::fromImage(aqx_hist));
  ui.a_hist_x->setScaledContents(true);
  ui.a_hist_y->setPixmap(QPixmap::fromImage(aqy_hist));
  ui.a_hist_y->setScaledContents(true);
  ui.a_hist_z->setPixmap(QPixmap::fromImage(aqz_hist));
  ui.a_hist_z->setScaledContents(true);
}

} // namespace vineslam_report
