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
ADD_OPTIONAL_DEPENDENCY("pinocchio >=  1.2.4")
option(USE_PINOCCHIO "compile with pinocchio")
# add eigen to compute the torques via rnea
ADD_OPTIONAL_DEPENDENCY("eigen3 >= 3.0.5")


ADD_EXECUTABLE(hrp2energy ${source_files})
# link pinocchio
IF ( '${PINOCCHIO_FOUND}' STREQUAL '1' )
IF ( '${USE_PINOCCHIO}' STREQUAL 'ON' )
  message (STATUS "## linking pinocchio ##" )
  # equivalent of "#define PINOCCHIO=linked"
  add_definitions ( -DPINOCCHIO="linked" )
  # link pinocchio
  PKG_CONFIG_USE_DEPENDENCY(hrp2energy pinocchio)
  # add eigen
  INCLUDE_DIRECTORIES(SYSTEM ${EIGEN3_INCLUDE_DIRS})
ENDIF ( '${USE_PINOCCHIO}' STREQUAL 'ON' )
ELSE ( '${PINOCCHIO_FOUND}' STREQUAL '1' )
    message (STATUS "pinocchio not found" )
ENDIF ( '${PINOCCHIO_FOUND}' STREQUAL '1' )
# link BOOST
TARGET_LINK_LIBRARIES(hrp2energy ${Boost_LIBRARIES})
