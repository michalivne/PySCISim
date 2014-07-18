CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(PHYSICAL_TRACKING_PROJECT)

set(CMAKE_OSX_ARCHITECTURES "x86_64")

# use debug mode when building Xcode
IF( CMAKE_GENERATOR STREQUAL "Xcode" )
    OPTION(DEBUG_MODE "Controls debug/optimization flags" ON)
ELSE ()
    OPTION(DEBUG_MODE "Controls debug/optimization flags" OFF)
ENDIF()

# Add CMake FIND* scripts
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# make global SWIG interface files available
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/swig)


# set path to include MacPorts rather thanl local python libraries.
IF(APPLE)
    SET(CMAKE_LIBRARY_PATH /opt/local/lib ${CMAKE_LIBRARY_PATH})

    ADD_DEFINITIONS(
    # C++ flags
#        -std=c++11 
#        -stdlib=libstdc++
#        -fno-strict-aliasing
    )

ENDIF (APPLE)

ADD_DEFINITIONS(
# optimization
#    -fPIC
#    -fvisibility-inlines-hidden
# warnings
    -Wall 
    -Wno-unused-label
    -Wno-conversion-null
    -Wno-write-strings
    -Wno-unused-but-set-variable
    -Wno-maybe-uninitialized
    -Wno-cpp
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

SET(SCISim_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../../SCISim) 

# include SCISim
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR}/include/rapidxml)
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/../../../SCISim/build/SDIC)
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/../../../SCISim/build/ThreeDRigidBodies)
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/../../../SCISim/build/ThreeDRigidBodiesUtils)

# add project directories
ADD_SUBDIRECTORY(PySCISim)
