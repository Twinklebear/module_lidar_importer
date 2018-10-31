#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>

#include <lasreader.hpp>
#include "common/sg/importer/Importer.h"
#include "common/sg/transferFunction/TransferFunction.h"
#include "common/sg/common/Common.h"
#include "common/sg/geometry/Spheres.h"
#include "ospcommon/containers/AlignedVector.h"
#include "ospcommon/vec.h"

using namespace ospray;
using namespace ospray::sg;

enum LIDAR_CLASSIFICATION {
  CREATED = 0,
  UNCLASSIFIED,
  GROUND,
  LOW_VEGETATION,
  MEDIUM_VEGETATION,
  HIGH_VEGETATION,
  BUILDING,
  NOISE,
  MODEL_KEY_POINT,
  WATER,
  OVERLAP_POINT,
  RESERVED
};
LIDAR_CLASSIFICATION classify_point(uint8_t class_attrib){
  assert(class_attrib < 32);
  switch (class_attrib){
    case 0: return CREATED;
    case 1: return UNCLASSIFIED;
    case 2: return GROUND;
    case 3: return LOW_VEGETATION;
    case 4: return MEDIUM_VEGETATION;
    case 5: return HIGH_VEGETATION;
    case 6: return BUILDING;
    case 7: return NOISE;
    case 8: return MODEL_KEY_POINT;
    case 9: return WATER;
    case 10: return RESERVED;
    case 11: return RESERVED;
    case 12: return OVERLAP_POINT;
    default: return RESERVED;
  }
}

void importLAS(const std::shared_ptr<Node> world, const FileName fileName){
  LASreadOpener read_opener;
  read_opener.set_file_name(fileName.c_str());
  LASreader *reader = read_opener.open();
  if (!reader){
    std::cout << "ImportLAS Error: Failed to open: " << fileName << ", skipping\n";
    return;
  }

  bool has_color = reader->header.point_data_format == 2
    || reader->header.point_data_format == 3
    || reader->header.point_data_format == 5;

  std::cout << "LiDAR file '" << fileName
    << "' contains " << reader->npoints << " points "
    << (has_color ? "with" : "without") << " color attributes\n"
    << "min: ( " << reader->get_min_x()
    << ", " << reader->get_min_y()
    << ", " << reader->get_min_z() << " )\n"
    << "max: ( " << reader->get_max_x()
    << ", " << reader->get_max_y()
    << ", " << reader->get_max_z() << " )\n";

  const vec3f min_pt(reader->get_min_x(), reader->get_min_y(), reader->get_min_z());
  const vec3f max_pt(reader->get_max_x(), reader->get_max_y(), reader->get_max_z());
  const vec3f diagonal = max_pt - min_pt;

  ospcommon::containers::AlignedVector<vec3f> points;
  ospcommon::containers::AlignedVector<vec4uc> colors;
  points.reserve(reader->npoints);
  colors.reserve(reader->npoints);
  int num_noise = 0;
  const float inv_max_color = 1.0f / std::numeric_limits<uint16_t>::max();
  while (reader->read_point()){
    // Points classified as low point are noise and should be discarded
    if (classify_point(reader->point.get_classification()) == NOISE){
      ++num_noise;
      continue;
    }
    reader->point.compute_coordinates();
    // Re-scale points to a better precision range for floats
    const vec3f p = vec3f(reader->point.coordinates[0], reader->point.coordinates[1],
        reader->point.coordinates[2]) - min_pt - diagonal * 0.5f;

    const uint16_t *rgba = reader->point.get_rgb();
    vec3f c;
    if (has_color){
      c = vec3f(rgba[0] * inv_max_color, rgba[1] * inv_max_color, rgba[2] * inv_max_color);
    }
    else {
      c = vec3f(1.0);
    }
    points.push_back(p);
    colors.push_back(vec4uc(c.x * 255.0, c.y * 255.0, c.z * 255.0, 255));
  }
  std::cout << "Discarded " << num_noise << " noise classified points\n";
  reader->close();
  delete reader;

  auto lidarGeom = createNode(fileName, "Spheres")->nodeAs<Spheres>();
  lidarGeom->createChild("bytes_per_sphere", "int", int(sizeof(vec3f)));
  lidarGeom->createChild("offset_center", "int", int(0));
  lidarGeom->createChild("radius", "float", 0.5f);

  auto materials = lidarGeom->child("materialList").nodeAs<MaterialList>();
  materials->item(0)["Ks"] = vec3f(0.f);

  auto spheres = std::make_shared<DataVectorT<vec3f, OSP_RAW>>();
  spheres->setName("spheres");
  spheres->v = std::move(points);

  auto colorData = std::make_shared<DataVectorT<vec4uc, OSP_UCHAR4>>();
  colorData->setName("color");
  colorData->v = std::move(colors);

  lidarGeom->add(spheres);
  lidarGeom->add(colorData);

  world->add(lidarGeom);
}

extern "C" OSPRAY_DLLEXPORT void ospray_init_module_lidar_import() {
  std::cout << "Loading LiDAR importer module\n";
}

OSPSG_REGISTER_IMPORT_FUNCTION(importLAS, lidar);

