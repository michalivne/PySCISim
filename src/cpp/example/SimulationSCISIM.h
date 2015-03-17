//This class handles loading and runing a SCISIM simulation
#ifndef _SIMULATIONSCISIM_H
#define _SIMULATIONSCISIM_H

//SCISIM includes
#include "ThreeDRigidBodies/ThreeDRigidBodySim.h"

#include "SCISim/Math/MathDefines.h"

#include "SCISim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "SCISim/UnconstrainedMaps/UnconstrainedMap.h"
#include "ThreeDRigidBodies/KinematicScripting.h"

#include "ThreeDRigidBodies/Geometry/RigidBodyBox.h"
#include "ThreeDRigidBodies/Geometry/RigidBodySphere.h"
#include "ThreeDRigidBodies/Geometry/RigidBodyTriangleMesh.h"

#include "ThreeDRigidBodies/UnconstrainedMaps/SplitHamMap.h"
#include "ThreeDRigidBodies/UnconstrainedMaps/ExponentialEulerMap.h"
#include "ThreeDRigidBodies/UnconstrainedMaps/ExactMap.h"
#include "ThreeDRigidBodies/UnconstrainedMaps/DMVMap.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/GROperator.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/LCPOperatorIpopt.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/LCPOperatorQL.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/LinearMDPOperatorIpopt.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/LinearMDPOperatorQL.h"
#include "SCISim/ConstrainedMaps/StabilizedImpactFrictionMap.h"
#include "SCISim/ConstrainedMaps/GeometricImpactFrictionMap.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/RestrictedSampleMDPOperatorIpopt.h"

#include "ThreeDRigidBodies/StaticGeometry/StaticPlane.h"
#include "ThreeDRigidBodies/Forces/NearEarthGravityForce.h"
#include "../include/QuadProg++.h"

#include <Eigen/StdVector>

//some useful typedefs
typedef Eigen::VectorXd PositionType;
typedef Eigen::MatrixXd TrajectoryGradientType;
typedef Eigen::VectorXd ParameterType;

//This class is reallya map between design state and SCISIM state
class SimulationSCISIM
{
	
	//just something for hanging onto constraints to test for initial position feasibility
	struct FeasibilityConstraint {
		unsigned int objectId;
		Vec3d n;
		double rhs;
		
	};
	
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SimulationSCISIM(const SimulationSCISIM &toCopy);
	
	virtual ~SimulationSCISIM();
	
	void operator=(const SimulationSCISIM &toAssign);
	
	void initStaticScene(unsigned int sceneId);
	
	void loadScene(const DesignState designState, unsigned int staticSceneId = 0);
	inline const DesignState & designState() const { return m_designState; }
	inline DesignState & designState() { return m_designState; }
	
	//Accessors
	double dt() const { return m_dt; }
	double t() const { return m_t; } //get the current sim time
	
	void setDt(double dt);
	void setRestitution(double cr) { m_CoR = cr; }
	void setFriction(double cf) {m_mu = cf; }
	
	//Get simulator
	const ThreeDRigidBodySim & sim() const { return m_sim; }
	
	//get current position and rotation for the specified object
	Matrix33sr rigidBodyRotation(unsigned int objIndex) const;
	Vector3s rigidBodyPosition(unsigned int objIndex) const;
	
	void setObjectParameters(unsigned int objId, const double *params, unsigned int size);
	void setObjectParameters(const double *params); //replace all params
	
	//restart from the begninnign
	void runTo(double time, bool timeCheck = false); //If simulation takes longer then 3*std, quit
	
	void fixInitialConditions();
	
	void reloadScene();
	
	double takeStep();
	
	void printRawGradient();
	
	inline unsigned int staticSceneId() const { return m_sceneId; }
	
	inline double restitution() const { return m_CoR; }
	inline double friction() const { return m_mu; }
	
	//make sure initial conditions are valid, if not move center of mass of objects so they are
	//this method is really simple it doesn't check for collisions between objects just between objects
	//and static planes that make up the test scenes
	void validInitialConditions();
	
	//SCISIM Variables
	// Simulation state
	UnconstrainedMap* m_unconstrained_map;
	ImpactOperator* m_impact_operator;
	FrictionOperator* m_friction_operator;
	ImpactFrictionMap* m_impact_friction_map;
	RigidBodyScriptingCallback* m_scripting_callback;
	
	double m_dt;
	double m_t;
	double m_mu;
	double m_CoR;
	
	// Current iteration of the solver
	unsigned int m_iteration;
	unsigned int m_numSCISIMObj;
	
	// Simulation
	ThreeDRigidBodySim m_sim;
	RigidBodySimState m_sim_state;
	
	std::vector<std::string> m_solvers;
	
	// SCISM rigidbody state parameters
	std::vector<Vector3s> m_xs;
	std::vector<Vector3s> m_vs;
	std::vector<scalar> m_Ms;
	std::vector<VectorXs> m_Rs;
	std::vector<Vector3s> m_omegas;
	std::vector<Vector3s> m_I0s;
	std::vector<RigidBodyGeometry *> m_geometry;
	std::vector<bool> m_fixed;
	std::vector<int> m_geometryIndices;
	std::vector<StaticPlane> m_staticPlanes;
	
	// SCISIM desdyn state parameters
	std::vector<Vector3s> m_xs_desdyn_params;
	std::vector<Vector3s> m_vs_desdyn_params;
	std::vector<VectorXs> m_Rs_desdyn_params;
	std::vector<Vector3s> m_omegas_desdyn_params;
	
	//Store rotations of rigid body frames (no way to grab this stuff out of SCISIM so cache it here)
	std::vector<Matrix33sr> m_R0;
	std::vector<Vector3s> m_COM;
	//Mapping between design state and scissim stuff
	std::vector<DesignSCISIM> m_designToSCISIM;
	std::vector<VectorXs> m_shape;
	std::vector<VectorXs> m_shape_desdyn_params;
	unsigned int m_sceneId;
	
private:
};


#endif
