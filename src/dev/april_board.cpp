#include <boost/make_shared.hpp>
#include "imgui.h"

#include "util/image_loader.hpp"
#include "dev/april_board.hpp"
#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"

namespace dev {
AprilBoard::AprilBoard(std::string &data_path) {
  board = boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(6, 6, tag_size_, tag_spacing_);
  // 加载图像
  if (!util::LoadTextureFromFile(data_path + "/imgs/april_board.png", texture_id_, img_width_, img_height_)) {
    texture_id_ = 0;
  }
}

void AprilBoard::show() { is_show_window_ = true; }

void AprilBoard::draw_ui() {
  const double zero{0.};
  if (!is_show_window_) {
    return;
  }

  if (texture_id_) {
    ImGui::Begin("AprilBoard Setting", &is_show_window_, ImGuiWindowFlags_AlwaysAutoResize);
    if (ImGui::DragScalar("tagSize(m)", ImGuiDataType_Double, &tag_size_, 0.01, &zero, nullptr, "%.3f")) {
      board->set_params(tag_size_, tag_spacing_);
    }
    if (ImGui::DragScalar("tagSpacing", ImGuiDataType_Double, &tag_spacing_, 0.01, &zero, nullptr, "%.3f")) {
      board->set_params(tag_size_, tag_spacing_);
    }

    ImGui::SameLine();
    ImGui::Spacing();
    ImGui::SameLine();
    ImGui::Spacing();
    ImGui::SameLine();

    if (ImGui::Button("update params")) {
      board->updata_params();
    }

    ImGui::Separator();
    ImGui::Image((void *)(intptr_t)texture_id_, ImVec2(float(img_width_), float(img_height_)));
    ImGui::End();
  }
}

}  // namespace dev