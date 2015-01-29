/*
 * A Python wrapper to SCISim library.
 */

#ifndef __PYSCISIM_H__
#define __PYSCISIM_H__

#include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
#include "ThreeDRigidBodiesQt4/Window.h"
#include "ThreeDRigidBodiesQt4/GLWidget.h"
#include "ThreeDRigidBodiesQt4/ContentWidget.h"
#include "ThreeDRigidBodies/Forces/Force.h"

#include <QApplication>
#include <QDesktopWidget>

#include <cstring>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <utility>

////////////////////////////////////////////
// SCISimWindow
////////////////////////////////////////////

typedef std::pair <VectorXs, int> ContactNormalElement;
typedef std::vector< ContactNormalElement > ContactNormalVec;

class SCISimApp {
public:
	SCISimApp(bool process_Qt_events=true);
	~SCISimApp();

	void run(const std::string& xml_scene_file_name="");

	// simulation interface
	void openScene(const std::string& xml_scene_file_name, unsigned fps=30, bool render_at_fps=true, bool lock_camera=false );

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
	unsigned int getSim_collisions();
	double getSim_collisions_penetration_depth();

	MatrixXs getSimState_M();
	VectorXs getSimState_q();
	VectorXs getSimState_v();
	VectorXs getSimState_T();
	VectorXs getSimState_U();
	VectorXs getSimState_p();
	VectorXs getSimState_L();
	int getSimState_nbodies();

	// update sim state
	void setSimState_q(const VectorXs& q);
	void setSimState_v(const VectorXs& v);

	// getter/setter of simulation state with rotation represented as angle/axis
	VectorXs get_x_from_q(const VectorXs& q); // converts q to x
    VectorXs get_q_from_x(const VectorXs& x); // converts x to q
	VectorXs interpolate_x(const VectorXs& x0, const VectorXs& x1, scalar t); // interpolate x
    VectorXs get_dxdt_from_x(const VectorXs& x0, const VectorXs& x1, double h);
    VectorXs get_dxdt_from_x(const VectorXs& x0, const VectorXs& x1, double h,
                             int nbodies);

	VectorXs get_x();
	VectorXs get_v();

	void set_x(const VectorXs& x);
	void set_v(const VectorXs& dxdt);
    // set dx/dt from two x vectors and time step h
    void set_dxdt(const VectorXs& x0, const VectorXs& x1, double h);

    // get a list of contact points and the corresponding normal
    void get_contacts_normal_and_body_ind( ContactNormalVec& normal_and_body_ind_vec );
    // iterates over all active contacts, and pushes bodies until there is no contact
    void resolve_contact();
    
	ThreeDRigidBodySim* getSim_sim();
	RigidBodySimState* getSim_sim_state_backup();

	void resetQt();

protected:
	void centerWindow();
	GLWidget* get_gl_widget();

protected:
	QApplication* app;
	Window* window;
	GLWidget* m_gl_widget;

	double time, T, U;
	Eigen::Vector3d p;
	Eigen::Vector3d L;

	unsigned int number_of_collisions;
	double collision_penetration_depth;

	bool process_Qt_events;
	static int Qt_argc;
	static char *Qt_argv[];
};

#endif // __PYSCISIM_H__
