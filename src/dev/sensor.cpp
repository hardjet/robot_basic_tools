#include <Eigen/Geometry>

#include "imgui.h"
#include "portable-file-dialogs.h"

#include "glk/mesh.hpp"
#include "glk/loaders/ply_loader.hpp"
#include "glk/glsl_shader.hpp"
#include "glk/primitives/primitives.hpp"

#include "dev/sensor.hpp"
#include "dev/util.hpp"

namespace dev {

// 打开配置文件夹默认路径
std::string config_default_path{};
// 打开数据文件夹默认路径
std::string data_default_path{};
// 设备名称
const std::string dev_type_str[] = {"UNDEF", "CAMERA", "LASER", "LIDAR", "IMU"};//NOLINT

uint32_t Sensor::sensors_unique_id = 0;

void Sensor::show() { b_show_window_ = true; }

bool Sensor::load_model() {
  // 选择加载文件路径
  std::vector<std::string> filters = {"3d model file (.ply)", "*.ply"};
  std::unique_ptr<pfd::open_file> dialog(new pfd::open_file("choose file", data_default_path, filters));
  while (!dialog->ready()) {
    usleep(1000);
  }

  auto file_paths = dialog->result();
  if (!file_paths.empty()) {
    glk::PLYLoader ply_model(file_paths[0]);
    if (ply_model.vertices.empty() || ply_model.normals.empty() || ply_model.indices.empty()) {
      std::string msg = "load 3d model from [" + file_paths[0] + "] failed!";
      show_pfd_info("load 3d model", msg);
      return false;
    } else {
      ply_model_ptr_ = std::make_unique<glk::Mesh>(ply_model.vertices, ply_model.normals, ply_model.indices);
      std::string msg = "load 3d model from [" + file_paths[0] + "] ok!";
      show_pfd_info("load 3d model", msg);
    }
  }
  return true;
}

Sensor::~Sensor() {
  if (ply_model_ptr_) {
    ply_model_ptr_->free();
  }
}

void Sensor::free_model() {
  if (ply_model_ptr_) {
    ply_model_ptr_->free();
    ply_model_ptr_ = nullptr;
  }
}

void Sensor::draw_status() {
  if (is_online_) {
    ImGui::TextColored(ImVec4(COLOR_ONLINE[0], COLOR_ONLINE[1], COLOR_ONLINE[2], 1.0f), "·");
  } else {
    ImGui::TextColored(ImVec4(COLOR_OFFLINE[0], COLOR_OFFLINE[1], COLOR_OFFLINE[2], 1.0f), "·");
  }
}

void Sensor::draw_gl_coordinate_system(glk::GLSLShader& shader) const{
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.rotate(T_.block<3, 3>(0, 0));
  T.pretranslate(T_.block<3, 1>(0, 3));

  // 画坐标系
  shader.set_uniform("color_mode", 2);
  shader.set_uniform("model_matrix", (T * Eigen::UniformScaling<float>(0.05f)).matrix());
  const auto& coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
  coord.draw(shader);
}

void Sensor::draw_data_color_selector() {
  ImGui::ColorEdit3("##color", data_color_.data(), ImGuiColorEditFlags_NoInputs);
}

}  // namespace dev