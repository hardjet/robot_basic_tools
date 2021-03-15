#include "GL/gl3w.h"

#include "glk/glsl_shader.hpp"
#include "glk/simple_lines.hpp"

namespace glk {

SimpleLines::SimpleLines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices,
                         const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors,
                         const std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>>& infos)
    : num_vertices(vertices.size()) {
  vao = vbo = cbo = ibo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 3, vertices.data(), GL_STATIC_DRAW);

  if (!colors.empty()) {
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * colors.size() * 4, colors.data(), GL_STATIC_DRAW);
  }

  if (!infos.empty()) {
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ARRAY_BUFFER, ibo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(int) * infos.size() * 4, infos.data(), GL_STATIC_DRAW);
  }

  /*
  std::vector<int> indices;
  for(int i = 0; i < vertices_ext.size(); i += 4) {
    indices.push_back(i);
    indices.push_back(i + 1);
    indices.push_back(i + 2);
    indices.push_back(i + 1);
    indices.push_back(i + 3);
    indices.push_back(i + 2);
  }
  */

  // std::vector<int> sub_indices = {0, 1, 4, 1, 5, 4, 4, 5, 2, 5, 3, 2, 2, 3, 6, 3, 6, 7, 6, 7, 0, 7, 1, 0};
  //
  // std::vector<int> indices;
  // for (int i = 0; i < vertices_ext.size(); i += 8) {
  //   for (int j = 0; j < sub_indices.size(); j++) {
  //     indices.push_back(sub_indices[j] + i);
  //   }
  // }
  // num_indices = indices.size();
  //
  // glGenBuffers(1, &ebo);
  // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(), indices.data(), GL_STATIC_DRAW);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

SimpleLines::~SimpleLines() {
  glDeleteBuffers(1, &vbo);
  if (cbo) {
    glDeleteBuffers(1, &cbo);
  }
  if (ibo) {
    glDeleteBuffers(1, &ibo);
  }

  // glDeleteBuffers(1, &ebo);
  glDeleteVertexArrays(1, &vao);
}

void SimpleLines::draw(glk::GLSLShader& shader) const {
  GLint position_loc = shader.attrib("vert_position");
  GLint color_loc = 0;
  GLint info_loc = 0;

  glBindVertexArray(vao);

  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

  if (cbo) {
    color_loc = shader.attrib("vert_color");
    glEnableVertexAttribArray(color_loc);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glVertexAttribPointer(color_loc, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
  }

  if (ibo) {
    info_loc = shader.attrib("vert_info");
    glEnableVertexAttribArray(info_loc);
    glBindBuffer(GL_ARRAY_BUFFER, ibo);
    glVertexAttribIPointer(info_loc, 4, GL_INT, 0, nullptr);
  }

  // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  // glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, nullptr);
  // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  // 画直线
  glDrawArrays(GL_LINES, 0, num_vertices);

  glDisableVertexAttribArray(position_loc);

  if (cbo) {
    glDisableVertexAttribArray(color_loc);
  }
  if (ibo) {
    glDisableVertexAttribArray(info_loc);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}
}  // namespace glk