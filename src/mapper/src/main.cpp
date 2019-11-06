#include "../include/mapper/main_window.hpp"

int main(int argc, char** argv)
{
	QApplication app(argc, argv);

  Mapper m(argc, argv);
	MainWindow w(&m);
	w.show();
	app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  QObject::connect(&m, SIGNAL(init_done()), &w, SLOT(init_done_slot()));
	int result = app.exec();

	return result;
}
