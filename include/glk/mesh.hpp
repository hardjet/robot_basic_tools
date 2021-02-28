#ifndef GLK_MESH_HPP
#define GLK_MESH_HPP

#include <vector>
#include <Eigen/Core>

#include "GL/gl3w.h"
#include "glk/drawble.hpp"

namespace glk {

/**
 * @brief Mesh class
 *
 */
class Mesh : public Drawable {
 public:
  Mesh(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &vertices,
       const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &normals,
       const std::vector<int> &indices);

  void free() override;

  void draw(glk::GLSLShader &shader) const override;

 private:
  Mesh(const Mesh &);

  Mesh &operator=(const Mesh &);

 private:
  int num_vertices;
  int num_indices;

  GLuint vao{};
  GLuint vbo{};
  GLuint nbo{};
  GLuint ebo{};
};

}  // namespace glk

#endif