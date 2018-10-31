# Module LiDAR Importer

A scenegraph importer module for OSPRay to support LiDAR data
(LAS and LAZ formats). Requires [LASTools](https://rapidlasso.com/lastools/)
to import the LAS/LAZ files.

To build OSPRay with this module, clone the repo under your "modules" directory
in the OSPRay source, and pass `-DOSPRAY_MODULE_LIDAR_IMPORTER=ON` to enable
the module. You can also hint at where to find LASTools, by passing
`-DLASTools_DIR=<path>`. The path for LASTools should be to the root of the
unzipped LASTools directory.

