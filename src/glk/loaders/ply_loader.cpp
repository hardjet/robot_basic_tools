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

  // 打印格式、元素、属性等信息
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
  bool gotVerts = false, gotFaces = false, gotNormals = false;
  while (reader.has_element() && (!gotVerts || !gotFaces)) {
    // 加载vertex
    if (reader.element_is(miniply::kPLYVertexElement) && reader.load_element() && reader.find_pos(indexes)) {
      vertices.resize(reader.num_rows());
      reader.extract_properties(indexes, 3, miniply::PLYPropertyType::Float, vertices.data());
      // 加载纹理
      // if (reader.find_texcoord(indexes)) {
      //   reader.extract_properties(indexes, 2, miniply::PLYPropertyType::Float, texcoord.data());
      // }

      // 加载法向量
      if (reader.find_normal(indexes)) {
        normals.resize(reader.num_rows());
        reader.extract_properties(indexes, 3, miniply::PLYPropertyType::Float, normals.data());
        gotNormals = true;
      }
      gotVerts = true;
    } else if (reader.element_is(miniply::kPLYFaceElement) && reader.load_element() && reader.find_indices(indexes)) {
      // 检查是否都是三角形
      bool polys = reader.requires_triangulation(indexes[0]);
      if (polys && !gotVerts) {
        fprintf(stderr, "Error: need vertex positions to triangulate faces.\n");
        break;
      }
      // 提取三角形
      if (polys) {
        printf("need extract_triangles! num_triangles = %d\n", reader.num_triangles(indexes[0]));
        indices.resize(reader.num_triangles(indexes[0]) * 3);
        reader.extract_triangles(indexes[0], reinterpret_cast<const float*>(vertices.data()), vertices.size(),
                                 miniply::PLYPropertyType::Int, indices.data());
      } else {
        fprintf(stdout, "num_triangles: %d, sum_of_list_counts: %d\n", reader.num_rows(),
                reader.sum_of_list_counts(indexes[0]));
        indices.resize(reader.num_rows() * 3);
        printf("indices.size() = %zu\n", indices.size());
        // reader.extract_list_property(indexes[0], miniply::PLYPropertyType::Int, indices.data());
      }
      gotFaces = true;
    }

    if (gotVerts && gotFaces) {
      break;
    }
    reader.next_element();
  }

  if (!gotVerts || !gotFaces) {
    fprintf(stderr, "Error: load %s failed!\n", filename.c_str());
    vertices.clear();
    indices.clear();
  } else if (!gotNormals) {
    // // 没有法向量需要自己计算
    // NormalEstimater nest(vertices, indices);
    // normals = nest.normals;
  }
}

}  // namespace glk
