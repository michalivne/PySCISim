# ThreeDRigidBodiesQt4 CMakeLists
#
# Breannan Smith
# Last updated: 03/09/2014

include( CMakeSourceFiles.txt )

# Qt4 on Linux seems to require a C compiler
if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
  enable_language( C )
endif()

# Eigen3 is required
find_package( Eigen3 3.1 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

# RapidXML is required
find_package( RapidXML REQUIRED )
include_directories( ${RAPIDXML_INCLUDE_DIR} )

# OpenGL is required
find_package( OpenGL REQUIRED )
include_directories( ${OPENGL_INCLUDE_DIR} )

# Qt4 is required
find_package( Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED )
include( ${QT_USE_FILE} )


add_executable( threed_rigid_bodies_qt4 ${Headers} ${Sources} ${HWQ_MOC_CPP} )

set_target_properties( threed_rigid_bodies_qt4 PROPERTIES AUTOMOC TRUE )

target_link_libraries( threed_rigid_bodies_qt4 ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ThreeDRigidBodiesUtils ThreeDRigidBodies SDIC )

execute_process( COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/assets/3DRigidBodyScene ${CMAKE_CURRENT_BINARY_DIR}/assets )
