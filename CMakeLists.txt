# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

project( shape_segmentation )


cmake_minimum_required(VERSION 2.8.11)

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# include helper file
include( ${CGAL_USE_FILE} )


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

# VTK
find_package(VTK REQUIRED)

if ( NOT VTK_FOUND )

  message(STATUS "This project requires the VTK library, and will not be compiled.")

  return()  

endif()

# include helper file
include(${VTK_USE_FILE})
message(STATUS ${VTK_MAJOR_VERSION})  # remember to put vtk-library dir in LD_LIBRARY_PATH for this cmake crap to work

# include for local package

add_executable(my_program segmentation.cpp)
target_link_libraries(my_program CGAL::CGAL ${VTK_LIBRARIES})



