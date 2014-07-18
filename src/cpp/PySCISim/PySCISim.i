%module PySCISim
%{
#define SWIG_FILE_WITH_INIT
#include "PySCISim.h"

typedef double scalar;

#include "SDIC/UnconstrainedMaps/FlowableSystem.h"
#include "SDIC/Constraints/ConstrainedSystem.h"
#include "SDIC/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
#include "ThreeDRigidBodies/KinematicScripting.h"
#include "ThreeDRigidBodies/RigidBodySimState.h"
#include "ThreeDRigidBodies/Forces/Force.h"
#include "ThreeDRigidBodiesQt4/Window.h"
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

typedef double scalar;

%template(ForcesVector) std::vector< Force* >;

%include "PySCISim.h"

%include "SDIC/UnconstrainedMaps/FlowableSystem.h"
%include "SDIC/Constraints/ConstrainedSystem.h"
%include "SDIC/ConstrainedMaps/ImpactMaps/ImpactMap.h"
%include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
%include "ThreeDRigidBodies/KinematicScripting.h"
%include "ThreeDRigidBodies/RigidBodySimState.h"
%include "ThreeDRigidBodies/Forces/Force.h"
%include "ThreeDRigidBodiesQt4/Window.h"
