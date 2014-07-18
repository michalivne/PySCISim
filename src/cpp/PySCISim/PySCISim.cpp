/*
 * A Python wrapper to SCISim library.
 */

#include "PySCISim.h"

using namespace std;

////////////////////////////////////////////
// SCISimWindow
////////////////////////////////////////////

SCISimApp::SCISimApp() {
	m_gl_widget = get_gl_widget();
	updateSimData();
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
	// Make sure old window is destroyed
	reset_QT();

	//QApplication::setGraphicsSystem("opengl");
	int argc = 1;
	char *argv[] = { "PySCISim" };
	app = boost::shared_ptr < QApplication > (new QApplication(argc, argv));

	window = boost::shared_ptr < Window > (new Window);
	window->resize(window->sizeHint());
	window->setWindowTitle("Three Dimensional Rigid Body Simulation");
	centerWindow();

	window->show();
	window->raise();
	app->exec();

	// store new gl_widget
	m_gl_widget = get_gl_widget();

}

GLWidget* SCISimApp::get_gl_widget() {
	// Make sure old window is destroyed
	reset_QT();

	//QApplication::setGraphicsSystem("opengl");
	int argc = 1;
	char *argv[] = { "PySCISim" };
	app = boost::shared_ptr < QApplication > (new QApplication(argc, argv));

	window = boost::shared_ptr < Window > (new Window);
	window->resize(window->sizeHint());
	window->setWindowTitle("Three Dimensional Rigid Body Simulation");
	centerWindow();
	window->show();
	window->raise();

	return window->get_content_widget()->get_gl_widget();
}

void SCISimApp::openScene(const std::string& xml_scene_file_name) {
	m_gl_widget->openScene(xml_scene_file_name.c_str());
}

void SCISimApp::stepSystem() {
	m_gl_widget->stepSystem();
	updateSimData();
}

void SCISimApp::resetSystem() {
	m_gl_widget->resetSystem();
}

void SCISimApp::updateSimData() {
	m_gl_widget->getSimData(time, T, U, p, L);
}

double SCISimApp::getSim_time() {
	return time;
}

double SCISimApp::getSim_T() {
	return T;
}

double SCISimApp::getSim_U() {
	return U;
}

Eigen::Vector3d SCISimApp::getSim_p() {
	return p;
}

Eigen::Vector3d SCISimApp::getSim_L() {
	return L;
}

ThreeDRigidBodySim* SCISimApp::getSim_sim() {
	return m_gl_widget->get_sim();
}

RigidBodySimState* SCISimApp::getSim_sim_state_backup() {
	return m_gl_widget->get_sim_state_backup();
}
