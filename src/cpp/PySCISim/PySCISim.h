/*
 * A Python wrapper to SCISim library.
 */

#ifndef __PYSCISIM_H__
#define __PYSCISIM_H__

#include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
#include "ThreeDRigidBodiesQt4/Window.h"
#include "ThreeDRigidBodiesQt4/GLWidget.h"
#include "ThreeDRigidBodiesQt4/ContentWidget.h"

#include <QApplication>
#include <QDesktopWidget>

#include <cstring>
#include <boost/shared_ptr.hpp>

////////////////////////////////////////////
// SCISimWindow
////////////////////////////////////////////

class SCISimApp {
public:
	SCISimApp(bool process_Qt_events=true);
	~SCISimApp();

	void run(const std::string& xml_scene_file_name="");

	// simulation interface
	void openScene(const std::string& xml_scene_file_name);

	// Methods to control the solver
	void stepSystem();
	void resetSystem();

	// Doubles because the plotting library expects doubles...
	void updateSimData();

	double getSim_time();
	double getSim_T();
	double getSim_U();
	Eigen::Vector3d getSim_p();
	Eigen::Vector3d getSim_L();

	MatrixXs getSimState_M();
	VectorXs getSimState_q();
	VectorXs getSimState_v();


	ThreeDRigidBodySim* getSim_sim();
	RigidBodySimState* getSim_sim_state_backup();

	void resetQt();

protected:
	void centerWindow();
	GLWidget* get_gl_widget();

protected:
	boost::shared_ptr<QApplication> app;
	boost::shared_ptr<Window> window;
	GLWidget* m_gl_widget;

	double time, T, U;
	Eigen::Vector3d p;
	Eigen::Vector3d L;

	bool process_Qt_events;
	static int Qt_argc;
	static char *Qt_argv[];
};

#endif // __PYSCISIM_H__
