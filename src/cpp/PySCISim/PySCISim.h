/*
 * A Python wrapper to SCISim library.
 */

#ifndef __PYSCISIM_H__
#define __PYSCISIM_H__

#include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
#include "Window.h"

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

	void centerWindow();
	void run();

protected:
	boost::shared_ptr<QApplication> app;
	boost::shared_ptr<Window> window;
};


#endif // __PYSCISIM_H__
