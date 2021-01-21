#ifndef GLK_GL_CANVAS_CANVAS_HPP
#define GLK_GL_CANVAS_CANVAS_HPP

#include <memory>
#include <string>
#include <Eigen/Core>

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

/**
 * @brief OpenGL canvas for imgui
 *
 */
class GLCanvas {
 public:
  GLCanvas(const std::string& data_directory, const Eigen::Vector2i& size);

  bool ready() const;

  void reset_camera();
  void set_size(const Eigen::Vector2i& size);
  void mouse_control() const;

  void bind(bool clear_buffers = true) const;
  void unbind() const;

  void render_to_screen(int color_buffer_id = 0) const;

  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const;
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const;
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const;

  void show_shader_setting();

  void draw_ui();
  void show_projection_setting() const;

 public:
  Eigen::Vector2i size;
  std::unique_ptr<glk::GLSLShader> shader;
  std::unique_ptr<glk::FrameBuffer> frame_buffer;
  std::unique_ptr<glk::TextureRenderer> texture_renderer;

  std::unique_ptr<guik::CameraControl> camera_control;
  std::unique_ptr<guik::ProjectionControl> projection_control;

 private:
  bool show_window;
  float point_size;
  float min_z;
  float max_z;
  bool z_clipping;
};

}  // namespace guik

#endif