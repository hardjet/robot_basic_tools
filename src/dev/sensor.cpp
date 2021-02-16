#include "imgui.h"
#include "portable-file-dialogs.h"

#include "glk/mesh.hpp"
#include "glk/loaders/ply_loader.hpp"

#include "dev/sensor.hpp"
#include "dev/util.hpp"

namespace dev {

// 打开配置文件夹默认路径
std::string config_default_path{};
// 打开数据文件夹默认路径
std::string data_default_path{};

uint32_t Sensor::sensors_unique_id = 0;

void Sensor::show() { is_show_window_ = true; }

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
    if (ply_model.indices.empty()) {
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

void Sensor::draw_status() {
  if (is_online_) {
    ImGui::TextColored(ImVec4(COLOR_ONLINE[0], COLOR_ONLINE[1], COLOR_ONLINE[2], 1.0f), "·");
  } else {
    ImGui::TextColored(ImVec4(COLOR_OFFLINE[0], COLOR_OFFLINE[1], COLOR_OFFLINE[2], 1.0f), "·");
  }
}

}  // namespace dev