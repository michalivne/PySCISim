CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(PHYSICAL_TRACKING_PROJECT)

set(CMAKE_OSX_ARCHITECTURES "x86_64")

# use debug mode when building Xcode
IF( CMAKE_GENERATOR STREQUAL "Xcode" )
    OPTION(DEBUG_MODE "Controls debug/optimization flags" ON)
ELSE ()
    OPTION(DEBUG_MODE "Controls debug/optimization flags" OFF)
ENDIF()

# Base research path
SET(RESEARCH_PATH "${CMAKE_SOURCE_DIR}/../../..")

# Common files
SET(COMMON_PATH "${RESEARCH_PATH}/Common")

# CMake finder files
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${COMMON_PATH}/cmake")

# swig interface files
INCLUDE_DIRECTORIES("${COMMON_PATH}/swig")


# set path to include MacPorts rather thanl local python libraries.
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    MESSAGE("***  Using OS X definitions")
    SET(CMAKE_LIBRARY_PATH /opt/local/lib ${CMAKE_LIBRARY_PATH})

    ADD_DEFINITIONS(
    # C++ flags
        -std=c++11
#        -stdlib=libc++
#        -stdlib=libstdc++
#        -fno-strict-aliasing
    )

ENDIF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    MESSAGE("***  Using Linux definitions")
    ADD_DEFINITIONS(
    # C++ flags
        -fPIC
        -std=c++11
#        -stdlib=libc++
#        -stdlib=libstdc++
#        -fno-strict-aliasing
    )

ENDIF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")


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

SET(SCISim_INCLUDE_DIR ${RESEARCH_PATH}/SCISim) 

# include SCISim
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${SCISim_INCLUDE_DIR}/include/rapidxml)
LINK_DIRECTORIES(${SCISim_INCLUDE_DIR}/build/SCISim)
LINK_DIRECTORIES(${SCISim_INCLUDE_DIR}/build/ThreeDRigidBodies)
LINK_DIRECTORIES(${SCISim_INCLUDE_DIR}/build/ThreeDRigidBodiesUtils)

# add project directories
ADD_SUBDIRECTORY(PySCISim)
