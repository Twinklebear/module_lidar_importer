option(OSPRAY_MODULE_LIDAR_IMPORTER "Build LIDAR scenegraph importer module" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

if (OSPRAY_MODULE_LIDAR_IMPORTER)
  find_package(LASTools REQUIRED)
  include_directories(${LASTOOLS_INCLUDE_DIRS} ${LASLIB_INCLUDE_DIR})
  ospray_create_library(ospray_module_lidar_import
    import_lidar.cpp
  LINK
    ospray_sg
    ospray_common
    ${LASTOOLS_LIBRARIES}
  )
endif()

