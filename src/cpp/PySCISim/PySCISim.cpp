/*
 * A Python wrapper to SCISim library.
 */

#include "PySCISim.h"

using namespace std;

////////////////////////////////////////////
// SCISimWindow
////////////////////////////////////////////

SCISimApp::SCISimApp() {
}

SCISimApp::~SCISimApp() {
}

void SCISimApp::reset_QT() {
	window = boost::shared_ptr<Window>();
	app = boost::shared_ptr<QApplication>();
}

void SCISimApp::centerWindow() {
	QDesktopWidget* desktop = QApplication::desktop();

	const int screenWidth = desktop->screenGeometry().width();
	const int screenHeight = desktop->screenGeometry().height();

	const QSize windowSize = window->size();
	const int width = windowSize.width();
	const int height = windowSize.height();

	const int x = (screenWidth - width) / 2;
	const int y = (screenHeight - height) / 2;

	window->move(x, y);
}

void SCISimApp::run() {
	//QApplication::setGraphicsSystem("opengl");
	int argc = 1;
	char *argv[] = {"PySCISim"};
	app = boost::shared_ptr<QApplication>(new QApplication(argc, argv));

	window = boost::shared_ptr<Window>(new Window);
	window->resize(window->sizeHint());
	window->setWindowTitle("Three Dimensional Rigid Body Simulation");
	centerWindow();

	window->show();
	window->raise();
	app->exec();

	// Make sure old window is destroyed
	reset_QT();
}


