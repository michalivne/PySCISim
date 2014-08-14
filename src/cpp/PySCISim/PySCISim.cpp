/*
 * A Python wrapper to SCISim library.
 */

#include "PySCISim.h"

using namespace std;

////////////////////////////////////////////
// SCISimWindow
////////////////////////////////////////////
int SCISimApp::Qt_argc = 1;
char *SCISimApp::Qt_argv[] = { "PySCISim" };

SCISimApp::SCISimApp(bool process_Qt_events) :
		process_Qt_events(process_Qt_events) {
	collision_penetration_depth = 0.0;

	//QApplication::setGraphicsSystem("opengl");
	app = new QApplication(Qt_argc, Qt_argv);
	window = new Window;

	get_gl_widget();
	updateSimData();
}

SCISimApp::~SCISimApp() {
	if (app) {
		app->quit();
	}

	resetQt();

	delete app;
}

void SCISimApp::resetQt() {
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

void SCISimApp::run(const std::string& xml_scene_file_name) {
	// load a scene if given
	if (xml_scene_file_name != "") {
		openScene(xml_scene_file_name);
	}

	app->exec();

	// reload scene/clear window or else Qt crashes.
	if (xml_scene_file_name != "") {
		openScene(xml_scene_file_name);
	} else
		// store new gl_widget
		get_gl_widget();
}

GLWidget* SCISimApp::get_gl_widget() {
	// Make sure old window is destroyed
	resetQt();

	m_gl_widget = window->get_content_widget()->get_gl_widget();
	m_gl_widget->useOpenGL(process_Qt_events);

	if (!process_Qt_events) {
		window->hide();
		m_gl_widget->toggleHUD();
	} else {
		window->resize(window->sizeHint());
		window->setWindowTitle("Three Dimensional Rigid Body Simulation");
		//	centerWindow();
		window->show();
		window->raise();
	}

	return m_gl_widget;
}

void SCISimApp::openScene(const std::string& xml_scene_file_name) {
	get_gl_widget();
	m_gl_widget->openScene(xml_scene_file_name.c_str());
	updateSimData();
}

void SCISimApp::stepSystem() {
	// step simulation
	m_gl_widget->stepSystem();
	// read simulation variables
	updateSimData();
	// step Qt events loop
	if (process_Qt_events)
		app->processEvents();
}

void SCISimApp::resetSystem() {
	m_gl_widget->resetSystem();
}

void SCISimApp::updateSimData() {
	m_gl_widget->getSimData(time, T, U, p, L);
	number_of_collisions = m_gl_widget->get_sim()->computeNumberOfCollisions();
	// FIXME: the next function generates warnings that crash ParallelPython.
//	m_gl_widget->get_sim()->computeNumberOfCollisions(number_of_collisions,
//			collision_penetration_depth);
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

unsigned int SCISimApp::getSim_collisions() {
	return number_of_collisions;
}

double SCISimApp::getSim_collisions_penetration_depth() {
	return collision_penetration_depth;
}

ThreeDRigidBodySim* SCISimApp::getSim_sim() {
	return m_gl_widget->get_sim();
}

RigidBodySimState* SCISimApp::getSim_sim_state_backup() {
	return m_gl_widget->get_sim_state_backup();
}

MatrixXs SCISimApp::getSimState_M() {
	return getSim_sim()->getState().M();
}

VectorXs SCISimApp::getSimState_q() {
	return getSim_sim()->getState().q();
}

VectorXs SCISimApp::getSimState_v() {
	return getSim_sim()->getState().v();
}

VectorXs SCISimApp::getSimState_T() {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	int N = m_sim_state.nbodies();
	VectorXs T(N);

	VectorXs T_helper = 0.5
			* m_sim_state.v().cwiseProduct(m_sim_state.M() * m_sim_state.v());

	// sum angular and linear kinetic energy
	for (int body = 0; body < N; ++body) {
		T(body) = T_helper.segment<3>(3 * body).sum()
				+ T_helper.segment<3>(3 * N + 3 * body).sum();
	}

	return T;
}

VectorXs SCISimApp::getSimState_U() {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	int N = m_sim_state.nbodies();
	VectorXs U = VectorXs::Zero(N);

	for (int body = 0; body < N; ++body) {
		VectorXs cur_q(12);
		cur_q.segment<3>(0) = m_sim_state.q().segment<3>(3*body);
		cur_q.segment<9>(3) = m_sim_state.q().segment<9>(3*N+9*body);
		MatrixXs M = this->getSimState_M();
		SparseMatrixsc cur_M(6, 6);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) {
				cur_M.insert(i, j) = M(3*body+i, 3*body+j);
				cur_M.insert(i+3, j+3) = M(3*N+3*body+i, 3*N+3*body+j);
			}
		for (std::vector<Force*>::size_type i = 0; i < m_sim_state.forces().size();
				++i) {
			U(body) += m_sim_state.forces()[i]->computePotential(
					cur_q, cur_M);
		}
	}

	return U;
}

VectorXs SCISimApp::getSimState_p() {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	int N = m_sim_state.nbodies();
	VectorXs p(N * 3);

	for (int body = 0; body < N; ++body) {
		p.segment<3>(3 * body) = m_sim_state.getTotalMass(body)
				* m_sim_state.v().segment<3>(3 * body);
	}

	return p;
}

VectorXs SCISimApp::getSimState_L() {
	const RigidBodySimState& m_sim_state = getSim_sim()->getState();
	int N = m_sim_state.nbodies();
	VectorXs L = VectorXs::Zero(N * 3);

	// Contribution from center of mass
	for (int body = 0; body < m_sim_state.nbodies(); ++body) {
		L.segment<3>(3 * body) += m_sim_state.getTotalMass(body)
				* m_sim_state.q().segment<3>(3 * body).cross(
						m_sim_state.v().segment<3>(3 * body));
	}

	// Contribution from rotation about center of mass
	for (int body = 0; body < m_sim_state.nbodies(); ++body) {
		L.segment<3>(3 * body) += m_sim_state.getInertia(body)
				* m_sim_state.v().segment<3>(
						3 * m_sim_state.nbodies() + 3 * body);
	}

	return L;
}

