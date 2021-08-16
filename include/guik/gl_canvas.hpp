#ifndef GLK_GL_CANVAS_CANVAS_HPP
#define GLK_GL_CANVAS_CANVAS_HPP

#include <memory>
#include <string>
#include <utility>
#include <set>
#include <Eigen/Core>

#include "glk/text_renderer.hpp"

namespace glk {
class GLSLShader;
class FrameBuffer;
class TextureRenderer;
}  // namespace glk

namespace guik {
class CameraControl;
class ProjectionControl;
}  // namespace guik

namespace guik {

class Parameter {
 public:
  Parameter(std::string text, float x, float y, float scale, glm::vec3 color = glm::vec3(0.3, 0.3, 0.8))
      : text_(std::move(text)), bl_x_(x), bl_y_(y), scale_(scale), color_(color) {}

 public:
  std::string text_;
  float bl_x_, bl_y_, scale_;
  glm::vec3 color_;
};

/**
 * @brief OpenGL canvas for imgui
 *
 */
class GLCanvas {
 public:
  GLCanvas(const std::string& data_directory, const Eigen::Vector2i& size);

  bool ready() const;

  void reset_camera();
  void set_size(const Eigen::Vector2i& fb_size);
  void mouse_control() const;

  void bind(bool clear_buffers = true) const;
  void unbind() const;

  void render_to_screen(int color_buffer_id = 0);

  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const;
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const;
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const;

  void show_shader_setting();

  void draw_ui();
  void show_projection_setting() const;

  std::pair<Eigen::Matrix4f, Eigen::Matrix4f> transformation_matrices() const;
  Eigen::Vector2i window_size() const;
  double viewer_diatance() const;

 public:
  Eigen::Vector2i size;
  std::unique_ptr<glk::GLSLShader> shader;
  std::unique_ptr<glk::FrameBuffer> frame_buffer;
  std::unique_ptr<glk::TextureRenderer> texture_renderer;
  std::unique_ptr<glk::TextRenderer> text_renderer;
  std::vector<Parameter> text_renderer_params;

  std::unique_ptr<guik::CameraControl> camera_control;
  std::unique_ptr<guik::ProjectionControl> projection_control;

 private:
  bool show_window{false};
  float point_size;
  float min_z;
  float max_z;
  bool z_clipping;
};

}  // namespace guik

#endif