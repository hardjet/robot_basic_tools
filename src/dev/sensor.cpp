#include "imgui.h"

#include "glk/mesh.hpp"
#include "glk/loaders/ply_loader.hpp"

#include "dev/sensor.hpp"

#include <memory>

namespace dev {

// 打开配置文件夹默认路径
std::string config_default_path{};
// 打开数据文件夹默认路径
std::string data_default_path{};

uint32_t Sensor::sensors_unique_id = 0;

void Sensor::show() { is_show_window_ = true; }

bool Sensor::load_model(const std::string& ply_file_name) {
  glk::PLYLoader ply_model(ply_file_name);
  if (ply_model.indices.empty()) {
    return false;
  }

  ply_model_ptr_ = std::make_unique<glk::Mesh>(ply_model.vertices, ply_model.normals, ply_model.indices);
  return true;
}

void Sensor::draw_status() {
  if (is_online_) {
    ImGui::TextColored(ImVec4(COLOR_ONLINE[0], COLOR_ONLINE[1], COLOR_ONLINE[2], 1.0f), "·");
  } else {
    ImGui::TextColored(ImVec4(COLOR_OFFLINE[0], COLOR_OFFLINE[1], COLOR_OFFLINE[2], 1.0f), "·");
  }
}

}  // namespace dev