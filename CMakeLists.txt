# Copyright 2014
#
# Maximilien Naveau
#
# LAAS, CNRS
#
# Creation: 30/10/2008
#
cmake_minimum_required(VERSION 2.6)                

#Configuration du projet

project(MyProject)

set(EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

include_directories(src)

#Configuration de l'exécutable
file(
        GLOB_RECURSE
        source_files
        src/ComputeEnergy.cc
        src/ComputeEnergy.hh
        src/motors.cc
        src/motors.hh
)

add_executable(
        my_exe
        ${source_files}
)


# Optimisation flags when building with RELEASE
#----------------------------------------------
IF(CMAKE_BUILD_TYPE MATCHES "RELEASE")
  IF(CMAKE_COMPILER_IS_GNUCXX)
    SET (${PROJECT_NAME}_CXXFLAGS "-O3 -funroll-loops -frerun-loop-opt -fschedule-insns2 -frerun-cse-after-loop -falign-functions -falign-labels -falign-loops -falign-jumps -fexpensive-optimizations ${${PROJECT_NAME}_CXXFLAGS}")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDIF(CMAKE_BUILD_TYPE MATCHES "RELEASE")

# Profiling flags when building with DEBUG
#-----------------------------------------
IF(CMAKE_BUILD_TYPE MATCHES "DEBUG")
  IF(CMAKE_COMPILER_IS_GNUCXX)
    # SET (${PROJECT_NAME}_CXXFLAGS "-fprofile-arcs -ftest-coverage ${${PROJECT_NAME}_CXXFLAGS}")
    SET (${PROJECT_NAME}_CXXFLAGS "${${PROJECT_NAME}_CXXFLAGS}")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDIF(CMAKE_BUILD_TYPE MATCHES "DEBUG")
