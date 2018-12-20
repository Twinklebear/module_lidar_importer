# Module LiDAR Importer

A scenegraph importer module for OSPRay to support LiDAR data
(LAS and LAZ formats). Requires [LASTools](https://rapidlasso.com/lastools/)
to import the LAS/LAZ files.

To build OSPRay with this module, clone the repo under your "modules" directory
in the OSPRay source, and pass `-DOSPRAY_MODULE_LIDAR_IMPORTER=ON` to enable
the module. You can also hint at where to find LASTools, by passing
`-DLASTools_DIR=<path>`. The path for LASTools should be to the root of the
unzipped LASTools directory on Windows, or on Linux you should build and install
LASTools with CMake and pass the install directory path. When building LASTools
on Linux you'll also need to specify to use position independent code
when running cmake: `-DCMAKE_CXX_FLAGS=-fPIC`.

You can then load the module and use it to import LiDAR data into the
OSPRay example viewer via:

```
./ospExampleViewer --module lidar_import --import:lidar:<path to las/laz>
```

