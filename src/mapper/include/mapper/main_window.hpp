#pragma once

#include "../../utils/qnode.hpp"
#include "mapper_node.hpp"
#include "ui_main_window.h"
#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QtCore>
#include <QtGui>
#include <iostream>

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QNode* node, QWidget* parent = 0);
	~MainWindow();

	Ui::MainWindowDesign ui;

public Q_SLOTS:
  void on_init_clicked();
  void on_draw_map_clicked();

private:
	QNode* qnode;
};
