#include <vector>
#include <fstream>
#include <iostream>

#include "glk/mesh_utils.hpp"
#include "glk/loaders/miniply.h"
#include "glk/loaders/ply_loader.hpp"

namespace glk {

static const char* kFileTypes[] = {
    "ascii",
    "binary_little_endian",
    "binary_big_endian",
};

static const char* kPropertyTypes[] = {
    "char", "uchar", "short", "ushort", "int", "uint", "float", "double",
};

PLYLoader::PLYLoader(const std::string& filename) {
  miniply::PLYReader reader(filename.c_str());

  if (!reader.valid()) {
    std::cerr << "error: failed to open " << filename << std::endl;
    return;
  }

  printf("ply format %s %d.%d\n", kFileTypes[int(reader.file_type())], reader.version_major(), reader.version_minor());

  for (uint32_t i = 0, endI = reader.num_elements(); i < endI; i++) {
    const miniply::PLYElement* elem = reader.get_element(i);
    printf("element %s %u\n", elem->name.c_str(), elem->count);
    for (const miniply::PLYProperty& prop : elem->properties) {
      if (prop.countType != miniply::PLYPropertyType::None) {
        printf("property list %s %s %s\n", kPropertyTypes[uint32_t(prop.countType)],
               kPropertyTypes[uint32_t(prop.type)], prop.name.c_str());
      } else {
        printf("property %s %s\n", kPropertyTypes[uint32_t(prop.type)], prop.name.c_str());
      }
    }
  }

  uint32_t indexes[3];
  bool gotVerts = false, gotFaces = false;
  while (reader.has_element() && (!gotVerts || !gotFaces)) {
    // 加载vertex
    if (reader.element_is(miniply::kPLYVertexElement) && reader.load_element() && reader.find_pos(indexes)) {
      vertices.resize(reader.num_rows());
      reader.extract_properties(indexes, 3, miniply::PLYPropertyType::Float, vertices.data());
      // 加载纹理
      // if (reader.find_texcoord(indexes)) {
      //   reader.extract_properties(indexes, 2, miniply::PLYPropertyType::Float, texcoord.data());
      // }
      gotVerts = true;
    } else if (reader.element_is(miniply::kPLYFaceElement) && reader.load_element() && reader.find_indices(indexes)) {
      bool polys = reader.requires_triangulation(indexes[0]);
      if (polys && !gotVerts) {
        fprintf(stderr, "Error: need vertex positions to triangulate faces.\n");
        break;
      }
      if (polys) {
        indices.resize(reader.num_triangles(indexes[0]) * 3);
        reader.extract_triangles(indexes[0], reinterpret_cast<const float*>(vertices.data()), vertices.size(), miniply::PLYPropertyType::Int,
                                 indices.data());
      } else {
        fprintf(stderr, "Error: must be triangulate faces.\n");
        break;
      }
      gotFaces = true;
    }
    if (gotVerts && gotFaces) {
      break;
    }
    reader.next_element();
  }


  std::ifstream ifs(filename);
  if (!ifs) {
    std::cerr << "error: failed to open " << filename << std::endl;
    return;
  }

  int num_vertices = 0;
  int num_faces = 0;
  std::vector<std::string> properties;
  while (!ifs.eof()) {
    std::string line;
    std::getline(ifs, line);

    if (line.empty()) {
      continue;
    }

    std::stringstream sst(line);
    std::string token;

    if (line.find("element vertex") != std::string::npos) {
      sst >> token >> token >> num_vertices;
    }
    if (line.find("element face") != std::string::npos) {
      sst >> token >> token >> num_faces;
    }

    if (line.find("property float32") != std::string::npos) {
      std::string property;
      sst >> token >> token >> property;
      properties.push_back(property);
    }

    if (line.find("end") != std::string::npos) {
      break;
    }
  }

  vertices.resize(num_vertices);
  for (int i = 0; i < num_vertices; i++) {
    std::string line;
    std::getline(ifs, line);

    std::stringstream sst(line);
    sst >> vertices[i][0] >> vertices[i][1] >> vertices[i][2];
  }

  indices.resize(num_faces * 3);
  for (int i = 0; i < num_faces; i++) {
    std::string line;
    std::getline(ifs, line);

    int faces = 0;
    std::stringstream sst(line);
    sst >> faces >> indices[i * 3 + 2] >> indices[i * 3 + 1] >> indices[i * 3];

    if (faces != 3) {
      std::cerr << "error : only faces with three vertices are supported!!" << std::endl;
    }
  }

  NormalEstimater nest(vertices, indices);
  normals = nest.normals;
}

}  // namespace glk
