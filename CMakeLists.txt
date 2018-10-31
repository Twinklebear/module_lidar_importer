option(OSPRAY_MODULE_LIDAR_IMPORTER "Build LIDAR scenegraph importer module" OFF)

list(APPEND CMAKE_MODULE_PATH "cmake/")

if (OSPRAY_MODULE_LIDAR_IMPORTER)
  find_package(LASTools REQUIRED)
  include_directories(${LASTOOLS_INCLUDE_DIRS})
  ospray_create_library(ospray_module_lidar_import
    import_lidar.cpp
  LINK
    ospray_sg
    ospray_common
    ${LASTOOLS_LIBRARIES}
  )
endif()

