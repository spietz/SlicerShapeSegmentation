# Shape Segmentation

Slicer extension for automatic segmentation of surface meshes (assigning labels to the triangles).
The underlying algorithm uses graph partitioning based on the "shape diameter function" from CGAL.

## Installation

The extension have not been packaged yet, so compile manually using cmake, which requires a [superbuild of Slicer](https://www.slicer.org/wiki/Documentation/4.0/Developers/Build_Instructions). Remember to change the path in the step-by-step instructions below. It further depends on CGAL which must be installed and available for cmake.

'''
git clone https://github.com/spietz/ShapeSegmentation.git
mkdir ShapeSegmentation/build
cd ShapeSegmentation/build
cmake DSlicer_DIR:PATH=/home/user/Slicer-SuperBuild-Debug/Slicer-build ..
make
'''

Once compiled add it to Slicer by browsing to the directory of 'ShapeSegmentation/build/lib/Slicer-*/cli-modules' when clicking 'Edit->Application Settings->Modules->Additional module paths->Add'


## References
* [CGAL documentation on surface segmentation](https://doc.cgal.org/latest/Surface_mesh_segmentation/index.html)
* Shapira, Lior, et al. "Consistent Mesh Partitioning and Skeletonisation Using the Shape Diameter Function." Visual Computer, vol. 24, no. 4, Springer-Verlag, 2008.

## TODO

* Add parameters for computing the shape diameter function.
* Compute shape diagmeter function seperately from segmentation.
* Allow any scalar field to guide the segmentation.
* Convert surface labels to volumetric label map to edit with the Segment Editor.
* Package 