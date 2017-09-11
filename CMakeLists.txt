# Copyright 2014
#
# Maximilien Naveau
#
# LAAS, CNRS
#
# Creation: 30/10/2008
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(CMakeDependentOption)

#Configuration du projet
PROJECT(ComputeHRP2energy)

SET(EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

INCLUDE_DIRECTORIES(./src/)

#Configuration de l'exécutable
FILE(
    GLOB_RECURSE source_files
    src/main.cc
    src/motors.cc
    src/explorefolder.cc
    src/experience.cc
    src/commonTools.cc
)

FILE(
    GLOB_RECURSE include_files
    src/main.hh
    src/motors.hh
    src/explorefolder.hh
    src/experience.hh
    src/commonTools.hh
)

# Search for boost
# Boost filesystem is used to read the datafiles.
SEARCH_FOR_BOOST()
# Use pinocchio to compute the torques during the motion
ADD_REQUIRED_DEPENDENCY("pinocchio >=  1.2.4")

ADD_EXECUTABLE(hrp2energy ${source_files})
PKG_CONFIG_USE_DEPENDENCY(hrp2energy pinocchio)
TARGET_LINK_LIBRARIES(hrp2energy ${Boost_LIBRARIES})
