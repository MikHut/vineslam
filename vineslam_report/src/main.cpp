#include <QtGui>
#include <QApplication>
#include "../include/vineslam_report/main_window.hpp"

int main(int argc, char** argv)
{
  QApplication                app(argc, argv);
  vineslam_report::MainWindow w(argc, argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
