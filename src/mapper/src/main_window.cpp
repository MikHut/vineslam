#include "../include/mapper/main_window.hpp"

MainWindow::MainWindow(QNode* node, QWidget* parent)
    : QMainWindow(parent), qnode(node)
{
	ui.setupUi(this);
  ui.tabWidget->setCurrentIndex(0);
	connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

	setWindowTitle(QApplication::translate("MainWindowDesign",
	                                       (*qnode).node_name.c_str(), 0));

  QString tmp;
  tmp = "Click on Init to setup the system.\n";
	ui.log->insertPlainText(tmp);
}

void MainWindow::on_init_clicked()
{
	(*qnode).init();
}

void MainWindow::init_done_slot()
{
  QString tmp;
  tmp = "All configs done. Ready to start!\n\n";
	ui.log->insertPlainText(tmp);
}

void MainWindow::on_draw_map_clicked()
{
	(*qnode).constructMap();
	cv::Mat map = (*qnode).exportMap();

	std::string text;
	(*qnode).retrieveLog(text);
  ui.log->clear();
	ui.log->insertPlainText(QString::fromUtf8(text.c_str()));

	QImage qmap = QImage(map.data, map.cols, map.rows, QImage::Format_RGB888);
	ui.map->setPixmap(QPixmap::fromImage(qmap));
  ui.map->setScaledContents(true);
}

void MainWindow::on_landmark_id_valueChanged(int id) {
	cv::Mat map = (*qnode).exportSingleMap(id);

  ui.log_estimation->clear();
  if(map.cols == 0 || map.rows == 0) {
    std::string error = "No such landmark id...";
	  ui.log_estimation->insertPlainText(QString::fromUtf8(error.c_str()));
    return;
  }

	std::string text;
	(*qnode).retrieveLog(text, id);
	ui.log_estimation->insertPlainText(QString::fromUtf8(text.c_str()));

	QImage qmap = QImage(map.data, map.cols, map.rows, QImage::Format_RGB888);
	ui.debug_map->setPixmap(QPixmap::fromImage(qmap));
  ui.debug_map->setScaledContents(true);
}

MainWindow::~MainWindow()
{
	delete qnode;
	(*this).destroy();
}
