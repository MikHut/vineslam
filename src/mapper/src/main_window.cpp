#include "../include/mapper/main_window.hpp"

MainWindow::MainWindow(QNode* node, QWidget* parent)
    : QMainWindow(parent), qnode(node)
{
	ui.setupUi(this);
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
	std::string text;
	(*qnode).retrieveLog(text);
	ui.log->insertPlainText(QString::fromUtf8(text.c_str()));

	(*qnode).constructMap();
	cv::Mat map = (*qnode).exportMap();

	QImage qmap = QImage(map.data, map.cols, map.rows, QImage::Format_RGB32);
	ui.map->setPixmap(QPixmap::fromImage(qmap));
}

MainWindow::~MainWindow()
{
	delete qnode;
	(*this).destroy();
}
