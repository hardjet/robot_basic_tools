
#include<opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <boost/make_shared.hpp>
#include "imgui.h"

#include "glk/glsl_shader.hpp"
#include "glk/primitives/primitives.hpp"
#include "glk/lines.hpp"

#include "util/image_loader.hpp"
#include "dev/chess_board.hpp"
#include "camera_model/chessboard/Chessboard.h"

namespace dev {
chessboard::chessboard(std::string& data_path) {
  cv::Size boardSize{tag_rows_, tag_cols_};
  //构造智能棋盘格
  board = boost::make_shared<camera_model::Chessboard>(boardSize, tag_size_);
  // 加载图像
  if (!util::LoadTextureFromFile(data_path + "/imgs/april_board.png", texture_id_, img_width_, img_height_)) {
    texture_id_ = 0;
  }
  T_ = Eigen::Matrix4f::Identity();
  board_lenght_ = tag_size_ * tag_cols_;
  board_height_ = tag_size_ * tag_rows_;
  chess_board_points_.clear();
  for (int i = 0; i < tag_rows_; ++i) {
    for (int j = 0; j < tag_cols_; ++j) {
      chess_board_points_.emplace_back(Eigen::Vector3d(j * tag_size_, i * tag_size_, 0.0));
    }
  }
  update_chessboard_edges();
}

void chessboard::update_chessboard_edges() {
  // edges的顶点
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  for (uint r = 0; r < tag_rows_; r++) {
    // 行线
      vertices.emplace_back(chess_board_points_.at(r * tag_cols_ ).cast<float>());
      vertices.emplace_back(chess_board_points_.at(r * tag_cols_ + tag_cols_-1).cast<float>());
  }
  for (uint c = 0; c < tag_cols_; c++) {
    // 列线
    vertices.emplace_back(chess_board_points_.at(c).cast<float>());
    vertices.emplace_back(chess_board_points_.at((tag_rows_  - 1) * tag_cols_ + c ).cast<float>());
  }
  chess_edges_ptr_.reset(new glk::Lines(0.0005f, vertices));
}

void chessboard::draw_gl(glk::GLSLShader& shader) {
  if (!b_show_3d_) {
    return;
  }
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.rotate(T_.block<3, 3>(0, 0));
  T.pretranslate(T_.block<3, 1>(0, 3));
  // 设置板子起点位置
  Eigen::Isometry3f T_B = Eigen::Isometry3f::Identity();
  T_B.pretranslate(Eigen::Vector3f(float(board_lenght_  / 2), float(board_height_/ 2 ), 0));

  // 增加上下板子边界
  Eigen::Matrix4f model_matrix =
      ((T * T_B) * Eigen::Scaling<float>(float(board_height_ + 0.04), float(board_lenght_ + 0.04), 0.0001)).matrix();

  shader.set_uniform("color_mode", 1);
  shader.set_uniform("model_matrix", model_matrix);
  shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 0.2f));
  shader.set_uniform("info_values", Eigen::Vector4i(0, 0, 0, 0));
  auto& cube = glk::Primitives::instance()->primitive(glk::Primitives::CUBE);
  cube.draw(shader);
  // // 画角点
  // shader.set_uniform("color_mode", 1);
  // // 设置显示颜色
  // shader.set_uniform("material_color", Eigen::Vector4f(0.f, 0.f, 0.f, 1.0f));
  // for (const auto& pt : april_points_) {
  //   draw_corner(T_, pt);
  // }
  // 画边
  shader.set_uniform("color_mode", 1);
  shader.set_uniform("model_matrix", T_);
  shader.set_uniform("material_color", Eigen::Vector4f(0.f, 0.f, 0.f, 1.0f));
  chess_edges_ptr_->draw(shader);

  // 画坐标系
  shader.set_uniform("color_mode", 2);
  shader.set_uniform("model_matrix", (T * Eigen::UniformScaling<float>(0.05f)).matrix());
  const auto& coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
  coord.draw(shader);
}

void chessboard::draw_ui() {
  const uint u_0{0};
  const uint u_2{2};
  const uint u_20{20};
  if (!b_show_window_) {
    return;
  }
  if (texture_id_) {
    ImGui::Begin("Chess Board Setting", &b_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::PushItemWidth(80);
    ImGui::DragScalar("tagRows   ", ImGuiDataType_U32, &tag_rows_, 1, &u_2, &u_20, "%d");
    ImGui::SameLine();
    ImGui::DragScalar("tagCols", ImGuiDataType_U32, &tag_cols_, 1, &u_2, &u_20, "%d");
    ImGui::SameLine();
    ImGui::DragScalar("tagSize(mm)", ImGuiDataType_Double, &tag_size_, 0.01, &u_0, &u_2, "%f");
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
      //重新生成board
      cv::Size boardSize{tag_rows_, tag_cols_};
      board.reset(new camera_model::Chessboard(boardSize, tag_size_));
      //构造board的大小
      board_lenght_ = tag_size_ * tag_cols_;
      board_height_ = tag_size_ * tag_rows_;
      //更新
      chess_board_points_.clear();
      //重新生成要显示的点
      for (int i = 0; i < tag_rows_; ++i) {
        for (int j = 0; j < tag_cols_; ++j) {
          chess_board_points_.emplace_back(Eigen::Vector3d(i * tag_size_, j * tag_size_, 0.0));
        }
      }
      // 更新3d显示
      update_chessboard_edges();
    }
    ImGui::Separator();
    ImGui::Image((void*)(intptr_t)texture_id_, ImVec2(float(img_width_), float(img_height_)));
    ImGui::End();
  }
}
}

