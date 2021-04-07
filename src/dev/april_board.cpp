#include <boost/make_shared.hpp>
#include "imgui.h"

#include "glk/glsl_shader.hpp"
#include "glk/drawble.hpp"
#include "glk/primitives/primitives.hpp"

#include "util/image_loader.hpp"
#include "dev/april_board.hpp"
#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"

namespace dev {
AprilBoard::AprilBoard(std::string& data_path) {
  board = boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(6, 6, tag_size_, tag_spacing_);
  // 加载图像
  if (!util::LoadTextureFromFile(data_path + "/imgs/april_board.png", texture_id_, img_width_, img_height_)) {
    texture_id_ = 0;
  }
  T_ = Eigen::Matrix4f::Identity();
  board_lenght_ = (1 + tag_spacing_) * tag_size_ * 6;
}

void AprilBoard::draw_gl(glk::GLSLShader& shader) {
  if (!b_show_3d_) {
    return;
  }

  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.rotate(T_.block<3, 3>(0, 0));
  T.pretranslate(T_.block<3, 1>(0, 3));
  // 设置板子起点位置
  Eigen::Isometry3f T_B = Eigen::Isometry3f::Identity();
  T_B.pretranslate(Eigen::Vector3f(board_lenght_ / 2, board_lenght_ / 2, 0));

  // 增加上下板子边界
  Eigen::Matrix4f model_matrix =
      ((T * T_B) * Eigen::Scaling<float>(board_lenght_ + 0.04, board_lenght_ + 0.2, 0.001)).matrix();

  shader.set_uniform("color_mode", 1);
  shader.set_uniform("model_matrix", model_matrix);
  shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 0.2f));
  shader.set_uniform("info_values", Eigen::Vector4i(0, 0, 0, 0));

  auto& cube = glk::Primitives::instance()->primitive(glk::Primitives::CUBE);
  cube.draw(shader);

  // 画坐标系
  shader.set_uniform("color_mode", 2);
  shader.set_uniform("model_matrix", (T * Eigen::UniformScaling<float>(0.05f)).matrix());
  const auto& coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
  coord.draw(shader);
}

void AprilBoard::draw_ui() {
  const double zero{0.};
  if (!b_show_window_) {
    return;
  }

  if (texture_id_) {
    ImGui::Begin("AprilBoard Setting", &b_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::PushItemWidth(80);
    if (ImGui::DragScalar("tagSize(m)", ImGuiDataType_Double, &tag_size_, 0.01, &zero, nullptr, "%.3f")) {
      board->set_params(tag_size_, tag_spacing_);
    }
    ImGui::SameLine();
    if (ImGui::DragScalar("tagSpacing", ImGuiDataType_Double, &tag_spacing_, 0.01, &zero, nullptr, "%.3f")) {
      board->set_params(tag_size_, tag_spacing_);
    }

    ImGui::PopItemWidth();

    // 使能数据显示
    ImGui::Dummy(ImVec2(303, 10));
    ImGui::SameLine();
    ImGui::Checkbox("##show_board", &b_show_3d_);
    // 提示设备状态
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("enable/disable show 3d board");
    }

    ImGui::SameLine();
    if (ImGui::Button("update params")) {
      board->updata_params();
      board_lenght_ = (1 + tag_spacing_) * tag_size_ * 6;
    }

    ImGui::Separator();
    ImGui::Image((void*)(intptr_t)texture_id_, ImVec2(float(img_width_), float(img_height_)));
    ImGui::End();
  }
}

}  // namespace dev