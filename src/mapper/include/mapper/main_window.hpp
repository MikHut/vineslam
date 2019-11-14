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
	void on_draw_histogram_clicked();
	void on_export_map_clicked();
	void init_done_slot();
	void on_landmark_id_valueChanged(int);
	void on_m_clicked();
	void on_dm_clicked();
	void on_cm_clicked();
	void on_mm_clicked();

private:
	QNode* qnode;

	float scale;
};
