
#include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>

#include "GL/gl3w.h"
#include "util/image_loader.hpp"

namespace util {

// Simple helper function to load an image into a OpenGL texture with common settings
bool LoadTextureFromFile(const std::string &filename, GLuint &img_texture, int &img_width, int &img_height) {
  // Load from file
  cv::Mat image_mat = cv::imread(filename);
  if (image_mat.empty()) {
    return false;
  }

  img_width = image_mat.cols;
  img_height = image_mat.rows;

  // cv::Mat image_fix_size;
  // // 转为指定为大小
  // if (image_mat.size().width != img_width || image_mat.size().height != img_height) {
  //   cv::resize(image_mat, image_fix_size, cv::Size{img_width, img_height});
  // } else {
  //   image_fix_size = image_mat;
  // }

  // Create a OpenGL texture identifier
  GLuint image_texture;
  glGenTextures(1, &image_texture);
  glBindTexture(GL_TEXTURE_2D, image_texture);

  // Setup filtering parameters for display
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  // Upload pixels into texture
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height, 0, GL_BGR, GL_UNSIGNED_BYTE, image_mat.data);

  img_texture = image_texture;
  return true;
}
}  // namespace util