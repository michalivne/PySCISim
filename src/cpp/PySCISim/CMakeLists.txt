CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(PYSCISIM)

FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})

FIND_PACKAGE(PythonLibs REQUIRED)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

FIND_PACKAGE(Numpy REQUIRED)
INCLUDE_DIRECTORIES(${NUMPY_INCLUDE_DIR})

SET(Boost_USE_STATIC_LIBS   ON)
SET(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE(Boost 1.48 REQUIRED system filesystem)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Eigen3 is required
FIND_PACKAGE( Eigen3 3.1 REQUIRED )
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

# RapidXML is required
#FIND_PACKAGE( RapidXML REQUIRED )
#INCLUDE_DIRECTORIES( ${RAPIDXML_INCLUDE_DIR} )

FIND_PACKAGE( Ipopt REQUIRED )
INCLUDE_DIRECTORIES( ${IPOPT_INC_DIRS} )

# OpenGL is required
FIND_PACKAGE( OpenGL REQUIRED )
INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIR} )

# Qt4 is required
FIND_PACKAGE( Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED )
INCLUDE( ${QT_USE_FILE} )

# include OpenMP if available
FIND_PACKAGE( OpenMP )
IF( OPENMP_FOUND )
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
ENDIF()

SET( ENV{BLA_VENDOR} ${SUGGESTED_BLAS_LAPACK} )
FIND_PACKAGE( LAPACK REQUIRED )
FIND_PACKAGE( BLAS REQUIRED )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LAPACK_LINKER_FLAGS} ${BLAS_LINKER_FLAGS} -lgfortran" )
SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LAPACK_LINKER_FLAGS} ${BLAS_LINKER_FLAGS} -lgfortran" )

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(PYSCISIM_FILES PySCISim.cpp PySCISim.h)

SET(ThreeDRigidBodiesQt4_FILES 
    ThreeDRigidBodiesQt4/ContentWidget.cpp
    ThreeDRigidBodiesQt4/GLWidget.cpp
    ThreeDRigidBodiesQt4/Window.cpp
    ThreeDRigidBodiesQt4/Rendering/StaticPlaneRenderer.cpp

    ThreeDRigidBodiesQt4/ContentWidget.h
    ThreeDRigidBodiesQt4/GLWidget.h
    ThreeDRigidBodiesQt4/Window.h
    ThreeDRigidBodiesQt4/Rendering/StaticPlaneRenderer.h

    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/EnergyWindow.cpp
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/QCustomPlot/qcustomplot.cpp
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/PerspectiveCameraController.cpp
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/OpenGL3DSphereRenderer.cpp
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/PlanarPortalRenderer.cpp
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/StaticCylinderRenderer.cpp
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/StapleRenderer.cpp
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/BodyGeometryRenderer.cpp
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/OrthographicCameraController.cpp
    
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/EnergyWindow.h
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/QCustomPlot/qcustomplot.h
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/PerspectiveCameraController.h
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/OpenGL3DSphereRenderer.h
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/PlanarPortalRenderer.h
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/StaticCylinderRenderer.h
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/StapleRenderer.h
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/Rendering/BodyGeometryRenderer.h
    ${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4/OrthographicCameraController.h

    ${HWQ_MOC_CPP}
    )

SET(PYSCISIM_LIBRARIES 
    ${CMAKE_THREAD_LIBS_INIT}
    ${IPOPT_LIB}
    ${QT_LIBRARIES} 
    ${OPENGL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${LAPACK_LIBRARIES}
         
    ThreeDRigidBodiesUtils 
    ThreeDRigidBodies 
    SCISim
)

# ThreeDRigidBodiesQt4 library
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesQt4)

ADD_LIBRARY(ThreeDRigidBodiesQt4 SHARED ${ThreeDRigidBodiesQt4_FILES})
SET_TARGET_PROPERTIES( ThreeDRigidBodiesQt4 PROPERTIES AUTOMOC TRUE )
TARGET_LINK_LIBRARIES(ThreeDRigidBodiesQt4 ${PYSCISIM_LIBRARIES})

# PySCISim library
ADD_LIBRARY(PySCISim_obj SHARED OBJECT ${PYSCISIM_FILES})
ADD_LIBRARY(PySCISim SHARED $<TARGET_OBJECTS:PySCISim_obj>)
TARGET_LINK_LIBRARIES(PySCISim ${PYSCISIM_LIBRARIES} ThreeDRigidBodiesQt4)

# PySCISim SWIG library
IF( CMAKE_GENERATOR STREQUAL "Xcode" )
    MESSAGE(WARNING "Skipping Python SWIG bindings in Xcode generator mode.")
ELSE ()
    SET_SOURCE_FILES_PROPERTIES(PySCISim.i PROPERTIES CPLUSPLUS ON)
    SWIG_ADD_MODULE(PySCISim python PySCISim.i  $<TARGET_OBJECTS:PySCISim_obj>)
    SWIG_LINK_LIBRARIES(PySCISim
        ThreeDRigidBodiesQt4 
        ${PYSCISIM_LIBRARIES}
        ${PYTHON_LIBRARIES}
    )
ENDIF()
        
