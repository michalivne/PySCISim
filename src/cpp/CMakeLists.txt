CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(PHYSICAL_TRACKING_PROJECT)

# use debug mode when building Xcode
IF( CMAKE_GENERATOR STREQUAL "Xcode" )
    OPTION(DEBUG_MODE "Controls debug/optimization flags" ON)
ELSE ()
    OPTION(DEBUG_MODE "Controls debug/optimization flags" OFF)
ENDIF()


SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

SET(OpenCV_DIR "/opt/local/lib/cmake")
# set path to include MacPorts rather thanl local python libraries.
IF(APPLE)
    SET(CMAKE_LIBRARY_PATH /opt/local/lib ${CMAKE_LIBRARY_PATH})

    ADD_DEFINITIONS(
    # C++ flags
#        -std=c++11 
#        -stdlib=libc++
#        -fno-strict-aliasing
    )

ENDIF (APPLE)

ADD_DEFINITIONS(
# optimization
#    -O3 
    -fPIC
    -fvisibility-inlines-hidden
# debug
#    -O0 
#    -g
# warnings
    -Wall 
#    -Wno-null-conversion 
#    -Wno-unused-label 
#    -Wno-overloaded-virtual 
#    -Wno-deprecated-declarations 
#    -Wno-tautological-constant-out-of-range-compare
#    -Wno-deprecated
)

IF (${DEBUG_MODE})
    MESSAGE("#######################################" )
    MESSAGE("# Building project in debug mode" )
    MESSAGE("#######################################" )
    ADD_DEFINITIONS(
    # debug
        -O0 
        -g
    )
ELSE ()
    ADD_DEFINITIONS(
    # optimization
        -O3 
    )
ENDIF()

SET(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} -builtin)
#SET(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} -builtin -includeall)

SET(SCISim_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../../SCISim) 

# include SCISim
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR}/ThreeDRigidBodies)
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR}/ThreeDRigidBodiesUtils)
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR}/include/)
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR}/include/rapidxml)

LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/../../../SCISim/build/SDIC)
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/../../../SCISim/build/ThreeDRigidBodies)
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/../../../SCISim/build/ThreeDRigidBodiesUtils)


# make global SWIG interface files available
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/swig)

# add project directories
ADD_SUBDIRECTORY(PySCISim)
