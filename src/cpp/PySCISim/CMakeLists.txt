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

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(PYSCISIM_FILES PySCISim.cpp PySCISim.h)

SET(ThreeDRigidBodiesQt4_FILES 
    ThreeDRigidBodiesQt4/ContentWidget.cpp
    ThreeDRigidBodiesQt4/EnergyWindow.cpp
    ThreeDRigidBodiesQt4/GLWidget.cpp
    ThreeDRigidBodiesQt4/QCustomPlot/qcustomplot.cpp
    ThreeDRigidBodiesQt4/CameraController.cpp
    ThreeDRigidBodiesQt4/Window.cpp
    ThreeDRigidBodiesQt4/Rendering/OpenGL3DSphereRenderer.cpp
    ThreeDRigidBodiesQt4/Rendering/PlanarPortalRenderer.cpp
    ThreeDRigidBodiesQt4/Rendering/StaticCylinderRenderer.cpp
    ThreeDRigidBodiesQt4/Rendering/StaticPlaneRenderer.cpp
    ThreeDRigidBodiesQt4/Rendering/StapleRenderer.cpp
    ThreeDRigidBodiesQt4/Rendering/BodyGeometryRenderer.cpp
    
    ThreeDRigidBodiesQt4/ContentWidget.h
    ThreeDRigidBodiesQt4/EnergyWindow.h
    ThreeDRigidBodiesQt4/GLWidget.h
    ThreeDRigidBodiesQt4/QCustomPlot/qcustomplot.h
    ThreeDRigidBodiesQt4/CameraController.h
    ThreeDRigidBodiesQt4/Window.h
    ThreeDRigidBodiesQt4/Rendering/OpenGL3DSphereRenderer.h
    ThreeDRigidBodiesQt4/Rendering/PlanarPortalRenderer.h
    ThreeDRigidBodiesQt4/Rendering/StaticCylinderRenderer.h
    ThreeDRigidBodiesQt4/Rendering/StaticPlaneRenderer.h
    ThreeDRigidBodiesQt4/Rendering/StapleRenderer.h
    ThreeDRigidBodiesQt4/Rendering/BodyGeometryRenderer.h
    
    ${HWQ_MOC_CPP}
    )

SET(PYSCISIM_LIBRARIES 
    ${CMAKE_THREAD_LIBS_INIT}
    ${IPOPT_LIB}
    ${QT_LIBRARIES} 
    ${OPENGL_LIBRARIES}
    ${Boost_LIBRARIES}
     
    ThreeDRigidBodiesUtils 
    ThreeDRigidBodies 
    SDIC
)

# ThreeDRigidBodiesQt4 library
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
        
