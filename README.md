# Shape Segmentation

Slicer extension for automatic segmentation of surface meshes (assigning labels to the triangles).
The underlying algorithm uses graph partitioning based on the "shape diameter function" from CGAL.

![Segmented hand](/Screenshots/hand.png)

## Installation

The extension have not been packaged yet, so compile manually using cmake, which requires a [superbuild of Slicer](https://www.slicer.org/wiki/Documentation/4.0/Developers/Build_Instructions). Remember to change the Slicer path in the instructions below. It further depends on CGAL which must be installed and available for cmake.

```
git clone https://github.com/spietz/SlicerShapeSegmentation.git
mkdir SlicerShapeSegmentation/build
cd SlicerShapeSegmentation/build
cmake DSlicer_DIR:PATH=/home/user/Slicer-SuperBuild-Debug/Slicer-build -DCMAKE_BUILD_TYPE=Release ..
make
```

Once compiled add it to Slicer by browsing to the directory of 'ShapeSegmentation/build/lib/Slicer-*/cli-modules' when clicking 'Edit->Application Settings->Modules->Additional module paths->Add'.


## References
* [CGAL documentation on surface segmentation](https://doc.cgal.org/latest/Surface_mesh_segmentation/index.html) and references therein.

## TODO
* Add parameters for computing the shape diameter function.
* Compute shape diameter function seperately from segmentation.
* Allow any scalar field to guide the segmentation.
* Convert surface labels to volumetric label map to edit with the Segment Editor.
* Package extension for use without Slicer superbuild.
