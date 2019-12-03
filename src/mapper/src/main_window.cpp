#include "../include/mapper/main_window.hpp"

MainWindow::MainWindow(QNode* node, QWidget* parent)
    : QMainWindow(parent), qnode(node)
{
	ui.setupUi(this);
	ui.tabWidget->setCurrentIndex(0);
  ui.draw_map->setEnabled(false);
  ui.draw_histogram->setEnabled(false);
  ui.filter_map->setEnabled(false);
  ui.export_map->setEnabled(false);
	connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

	setWindowTitle(QApplication::translate("MainWindowDesign",
	                                       (*qnode).node_name.c_str(), 0));

	QString tmp;
	tmp = "Click on Init to setup the system.\n";
	ui.log->insertPlainText(tmp);
  ui.map_width->setText("1000");
  ui.map_height->setText("1000");

  scale = 100;
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
  ui.init->setEnabled(false);
  ui.draw_map->setEnabled(true);
  ui.filter_map->setEnabled(true);
  ui.export_map->setEnabled(true);

  if((*qnode).histogramType() == true)
    ui.draw_histogram->setEnabled(true);
}

void MainWindow::on_draw_map_clicked()
{
  int w = ui.map_width->text().toInt();
  int h = ui.map_height->text().toInt();

	(*qnode).constructMap(scale, w, h);
	cv::Mat map = (*qnode).exportMap();

	std::string text;
	(*qnode).retrieveLog(text);
	ui.log->clear();
	ui.log->insertPlainText(QString::fromUtf8(text.c_str()));

	QImage qmap = QImage(map.data, map.cols, map.rows, QImage::Format_RGB888);
	ui.map->setPixmap(QPixmap::fromImage(qmap));
	ui.map->setScaledContents(true);
}

void MainWindow::on_draw_histogram_clicked() 
{
  int w = ui.map_width->text().toInt();
  int h = ui.map_height->text().toInt();

	(*qnode).constructMap(scale, w, h);
	cv::Mat hist = (*qnode).exportHistogram();

	std::string text;
	(*qnode).retrieveLog(text);
	ui.log->clear();
	ui.log->insertPlainText(QString::fromUtf8(text.c_str()));

	QImage qhist = QImage(hist.data, hist.cols, hist.rows, QImage::Format_RGB888);
	ui.map->setPixmap(QPixmap::fromImage(qhist));
	ui.map->setScaledContents(true);
}

void MainWindow::on_export_map_clicked() 
{
	std::string text = "Map saved to /home/user/map.txt\n";
	ui.log->clear();
	ui.log->insertPlainText(QString::fromUtf8(text.c_str()));

  (*qnode).saveMap();
}

void MainWindow::on_landmark_id_valueChanged(int id)
{
  int w = ui.map_width->text().toInt();
  int h = ui.map_height->text().toInt();

	cv::Mat map = (*qnode).exportSingleMap(id, scale, w, h);

	ui.log_estimation->clear();
	if (map.cols == 0 || map.rows == 0) {
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

void MainWindow::on_filter_map_clicked() 
{
  cv::Mat map = (*qnode).filterMap();

	QImage qmap = QImage(map.data, map.cols, map.rows, QImage::Format_RGB888);
	ui.map->setPixmap(QPixmap::fromImage(qmap));
	ui.map->setScaledContents(true);

	std::string text;
	(*qnode).retrieveLog(text);
	ui.log->clear();
	ui.log->insertPlainText(QString::fromUtf8(text.c_str()));
}

void MainWindow::on_m_clicked()
{
  scale = 1;
}

void MainWindow::on_dm_clicked()
{
  scale = 10;
}

void MainWindow::on_cm_clicked()
{
  scale = 100;
}

void MainWindow::on_mm_clicked()
{
  scale = 1000;
}

MainWindow::~MainWindow()
{
	delete qnode;
	(*this).destroy();
}
