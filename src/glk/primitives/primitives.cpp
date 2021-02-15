#include <iostream>
#include "glk/drawble.hpp"
#include "glk/mesh.hpp"
#include "glk/lines.hpp"
#include "glk/mesh_utils.hpp"

#include "glk/primitives/grid.hpp"
#include "glk/primitives/cube.hpp"
#include "glk/primitives/cone.hpp"
#include "glk/primitives/icosahedron.hpp"
#include "glk/primitives/coordinate_system.hpp"
#include "glk/loaders/ply_loader.hpp"
#include "glk/primitives/primitives.hpp"

namespace glk {

Primitives *Primitives::instance_ = nullptr;

const glk::Drawable &Primitives::primitive(PrimitiveType type) {
  if (meshes[type] == nullptr) {
    switch (type) {
      default:
        std::cerr << "error : unknown primitive type " << type << std::endl;
        break;
      case ICOSAHEDRON: {
        // 二十面体
        glk::Icosahedron icosahedron;
        glk::Flatize flat(icosahedron.vertices, icosahedron.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices));
      } break;
      case SPHERE: {
        // 球面
        glk::Icosahedron icosahedron;
        icosahedron.subdivide();
        icosahedron.subdivide();
        icosahedron.spherize();
        meshes[type].reset(new glk::Mesh(icosahedron.vertices, icosahedron.normals, icosahedron.indices));
      } break;
      case CUBE: {
        // 立方体
        glk::Cube cube;
        glk::Flatize flat(cube.vertices, cube.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices));
      } break;
      case CONE: {
        // 椎体
        glk::Cone cone;
        glk::Flatize flat(cone.vertices, cone.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices));
      } break;
      case GRID: {
        // 栅格
        glk::Grid grid;
        meshes[type].reset(new glk::Lines(0.01f, grid.vertices));
      } break;
      case BUNNY: {
        glk::PLYLoader ply("data/models/bunny.ply");
        meshes[type].reset(new glk::Mesh(ply.vertices, ply.normals, ply.indices));
      } break;
      case COORDINATE_SYSTEM: {
        glk::CoordinateSystem coord;
        meshes[type].reset(new glk::Lines(0.01f, coord.vertices, coord.colors));
      } break;
    }
  }

  return *meshes[type];
}
}  // namespace glk