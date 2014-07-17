%module PySCISim
%{
#define SWIG_FILE_WITH_INIT
#include "PySCISim.h"

#include "SDIC/UnconstrainedMaps/FlowableSystem.h"
#include "SDIC/Constraints/ConstrainedSystem.h"
#include "SDIC/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
%}

%include "numpy.i"

%init %{
    import_array();
%}

%include "typemaps.i"
%include "std_vector.i"
%include "std_string.i"
%include "std_list.i"
%include "std_map.i"

%include "common.i"

%include "PySCISim.h"

%include "SDIC/UnconstrainedMaps/FlowableSystem.h"
%include "SDIC/Constraints/ConstrainedSystem.h"
%include "SDIC/ConstrainedMaps/ImpactMaps/ImpactMap.h"
%include "ThreeDRigidBodies/ThreeDRigidBodySim.h"
