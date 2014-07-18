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
	SCISimApp();
	~SCISimApp();

	void run();

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

	ThreeDRigidBodySim* getSim_sim();
	RigidBodySimState* getSim_sim_state_backup();

protected:
	void reset_QT();
	void centerWindow();
	GLWidget* get_gl_widget();

protected:
	boost::shared_ptr<QApplication> app;
	boost::shared_ptr<Window> window;
	GLWidget* m_gl_widget;

	double time, T, U;
	Eigen::Vector3d p;
	Eigen::Vector3d L;
};

#endif // __PYSCISIM_H__
