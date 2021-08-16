#ifndef GLK_TEXTURE_HPP
#define GLK_TEXTURE_HPP

#include <vector>
#include <Eigen/Core>
#include "GL/gl3w.h"

namespace glk {

/**
 * @brief OpenGL texture wrapper
 *
 */
class Texture {
 public:
  Texture(const Eigen::Vector2i& size, GLuint internal_format, GLuint format,
          GLuint type)  // internal_format告诉OpenGL内部用什么格式存储和使用这个纹理数据 ， type是GL_RGBA的类型
      : width(size[0]), height(size[1]) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, size[0], size[1], 0, format, type, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  ~Texture() { glDeleteRenderbuffers(1, &texture); }

  GLuint id() const { return texture; }
  Eigen::Vector2i size() const { return Eigen::Vector2i{width, height}; }

  template <typename T>
  std::vector<T> read_pixels(GLuint format = GL_RGBA, GLuint type = GL_UNSIGNED_BYTE) const {
    std::vector<T> pixels(width * height * 4);
    glBindTexture(GL_TEXTURE_2D, texture);
    glGetTexImage(GL_TEXTURE_2D, 0, format, type, pixels.data());
    return pixels;
  }

 private:
  int width;
  int height;
  GLuint texture{};
};
}  // namespace glk

#endif