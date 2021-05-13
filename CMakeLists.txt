cmake_minimum_required(VERSION 3.13.4)

project(SurfSegmentCLI)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "https://github.com/spietz/ShapeSegmentation")
set(EXTENSION_CATEGORY "Surface Models")
set(EXTENSION_CONTRIBUTORS "Henrik Spietz")
set(EXTENSION_DESCRIPTION "Graph based segmentation of shapes using the 'shape diameter function' and CGAL. For details see https://doc.cgal.org/latest/Surface_mesh_segmentation/index.html and references therein.")
set(EXTENSION_ICONURL "https://github.com/spietz/ShapeSegmentation/ShapeSegmentation.png")
set(EXTENSION_SCREENSHOTURLS "https://github.com/spietz/ShapeSegmentation/ShapeSegmentation.png/Screenshots/hand.png")
set(EXTENSION_DEPENDS "NA") # Specified as a list or "NA" if no dependencies

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(Segmenter)
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_GENERATE_CONFIG})
include(${Slicer_EXTENSION_CPACK})
