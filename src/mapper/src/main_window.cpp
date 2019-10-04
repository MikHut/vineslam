#include "../include/mapper/main_window.hpp"

MainWindow::MainWindow(QNode* node, QWidget* parent)
    : QMainWindow(parent), qnode(node)
{
	ui.setupUi(this);
	connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

	setWindowTitle(QApplication::translate("MainWindowDesign",
	                                       (*qnode).node_name.c_str(), 0));
}

void MainWindow::on_init_clicked()
{
	(*qnode).init();
}

void MainWindow::on_draw_map_clicked() 
{
  std::string text;
  (*qnode).retrieveLog(text);
  ui.log->setText(QString::fromUtf8(text.c_str()));
}

MainWindow::~MainWindow()
{
	delete qnode;
	(*this).destroy();
}
