#pragma once

#include <vector>
#include <Eigen/Core>

#include "GL/gl3w.h"
#include "glk/drawble.hpp"

namespace glk {

/**
 * @brief A class to draw a set of simple lines
 *
 */
class SimpleLines : public Drawable {
 public:
  explicit SimpleLines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices,
                       const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors =
                           std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>(),
                       const std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>>& infos =
                           std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>>());
  ~SimpleLines() override;

  void draw(glk::GLSLShader& shader) const override;

 private:
  SimpleLines(const SimpleLines&);
  SimpleLines& operator=(const SimpleLines&);

 private:
  int num_vertices;
  // int num_indices;

  GLuint vao;  // vertex array object
  GLuint vbo;  // vertices
  GLuint cbo;  // colors
  GLuint ibo;  // infos
  // GLuint ebo{};  // elements
};
}  // namespace glk
