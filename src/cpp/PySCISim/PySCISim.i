%module PySCISim
%{
#define SWIG_FILE_WITH_INIT
#include "PySCISim.h"

typedef double scalar;

//#include "SCISim/UnconstrainedMaps/FlowableSystem.h"
//#include "SCISim/Constraints/ConstrainedSystem.h"
//#include "SCISim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
//#include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
//#include "ThreeDRigidBodies/KinematicScripting.h"
//#include "ThreeDRigidBodies/RigidBodySimState.h"
//#include "ThreeDRigidBodies/Forces/Force.h"
%}

%include "typemaps.i"
%include "numpy.i"
%include "eigen.i"

%init %{
    import_array();
%}
		
%include "std_vector.i"
%include "std_string.i"
%include "std_list.i"
%include "std_map.i"

%include "common.i"

// SWIG interface definitions
%ignore RigidBodySimState::getInertia;

%eigen_typemaps(Vector3s)
%eigen_typemaps(VectorXs)
%eigen_typemaps(MatrixXs)
%eigen_typemaps(Eigen::Vector3d)

typedef double scalar;

%template(ForcesVector) std::vector< Force* >;

%include "PySCISim.h"

//%include "SCISim/UnconstrainedMaps/FlowableSystem.h"
//%include "SCISim/Constraints/ConstrainedSystem.h"
//%include "SCISim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
//%include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
//%include "ThreeDRigidBodies/KinematicScripting.h"
//%include "ThreeDRigidBodies/RigidBodySimState.h"
//%include "ThreeDRigidBodies/Forces/Force.h"
