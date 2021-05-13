
#-----------------------------------------------------------------------------
set(MODULE_NAME Segmenter)
#-----------------------------------------------------------------------------

#
# SlicerExecutionModel
#
find_package(SlicerExecutionModel REQUIRED)
include(${SlicerExecutionModel_USE_FILE})

#
# CGAL and its components
#
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# include helper file
include( ${CGAL_USE_FILE} )

#
# Boost and its components
#
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

#
# VTK
#
find_package(VTK REQUIRED)

if ( NOT VTK_FOUND )

  message(STATUS "This project requires the VTK library, and will not be compiled.")

  return()  

endif()

# include helper file
include(${VTK_USE_FILE})
message(STATUS ${VTK_MAJOR_VERSION})

#-----------------------------------------------------------------------------
set(MODULE_INCLUDE_DIRECTORIES
  ${MRMLCore_INCLUDE_DIRS}
  )

set(MODULE_SRCS
  )

set(MODULE_TARGET_LIBRARIES
  MRMLCore
  CGAL::CGAL
  ${VTK_LIBRARIES}
  )

#-----------------------------------------------------------------------------
SEMMacroBuildCLI(
  NAME ${MODULE_NAME}
  TARGET_LIBRARIES ${MODULE_TARGET_LIBRARIES}
  INCLUDE_DIRECTORIES ${MODULE_INCLUDE_DIRECTORIES}
  ADDITIONAL_SRCS ${MODULE_SRCS}
  )

#-----------------------------------------------------------------------------
if(BUILD_TESTING)
  add_subdirectory(Testing)
endif()
