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

void Sensor::draw_gl_coordinate_system(glk::GLSLShader& shader, const std::shared_ptr<guik::GLCanvas>& canvas_ptr) const {

  std::pair<Eigen::Matrix4f, Eigen::Matrix4f> feedback = canvas_ptr->transformation_matrices();
  Eigen::Matrix4f view = feedback.first;
  Eigen::Matrix4f project = feedback.second;
  Eigen::Vector2i size = canvas_ptr->window_size();
  double dist = canvas_ptr->viewer_diatance();
  Eigen::Vector4f clipped = project * view * Eigen::Vector4f(T_(0, 3), T_(1, 3), T_(2, 3), T_(3, 3));

  float x = (static_cast<float>(size(0)) / 2.0f) + (clipped(0) / clipped(3) / 2.0f * static_cast<float>(size(0)));
  float y = (static_cast<float>(size(1)) / 2.0f) + (clipped(1) / clipped(3) / 2.0f * static_cast<float>(size(1))) - 20;
  float s = 0.35f - static_cast<float>(dist - 5.0f) * (0.35f / 5.0f);
  canvas_ptr->text_renderer_params.emplace_back(guik::Parameter(sensor_name, x, y, s, glm::vec3(1, 1, 1)));

  printf("clipped = [%f, %f, %f, %f]\n", clipped(0), clipped(1), clipped(2), clipped(3));
  printf("distance = %f, scale = %f\n", dist, s);
  printf("x = %f, y = %f\n\n", x, y);

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

//  std::cout << "T : " << std::endl;
//  std::cout << T(0, 0) << ", " << T(0, 1) << ", " << T(0, 2) << ", " << T(0, 3) << std::endl;
//  std::cout << T(1, 0) << ", " << T(1, 1) << ", " << T(1, 2) << ", " << T(1, 3) << std::endl;
//  std::cout << T(2, 0) << ", " << T(2, 1) << ", " << T(2, 2) << ", " << T(2, 3) << std::endl;
//  std::cout << T(3, 0) << ", " << T(3, 1) << ", " << T(3, 2) << ", " << T(3, 3) << std::endl;


//  Eigen::Matrix4f clipped = view * T_;
//  glm::mat4 tmp = glm::make_mat4(clipped.data()) * glm::ortho(0.0f, 1280.0f, 0.0f, 720.0f);
//
//  std::cout << "T_ : " << std::endl;
//  std::cout << T_(0, 0) << ", " << T_(0, 1) << ", " << T_(0, 2) << ", " << T_(0, 3) << std::endl;
//  std::cout << T_(1, 0) << ", " << T_(1, 1) << ", " << T_(1, 2) << ", " << T_(1, 3) << std::endl;
//  std::cout << T_(2, 0) << ", " << T_(2, 1) << ", " << T_(2, 2) << ", " << T_(2, 3) << std::endl;
//  std::cout << T_(3, 0) << ", " << T_(3, 1) << ", " << T_(3, 2) << ", " << T_(3, 3) << std::endl;
//
//  std::cout << "clipped : " << std::endl;
//  std::cout << clipped(0, 0) << ", " << clipped(0, 1) << ", " << clipped(0, 2) << ", " << clipped(0, 3) << std::endl;
//  std::cout << clipped(1, 0) << ", " << clipped(1, 1) << ", " << clipped(1, 2) << ", " << clipped(1, 3) << std::endl;
//  std::cout << clipped(2, 0) << ", " << clipped(2, 1) << ", " << clipped(2, 2) << ", " << clipped(2, 3) << std::endl;
//  std::cout << clipped(3, 0) << ", " << clipped(3, 1) << ", " << clipped(3, 2) << ", " << clipped(3, 3) << std::endl;
//
//  std::cout << "tmp : " << std::endl;
//  std::cout << tmp[0][0] << ", " << tmp[0][1] << ", " << tmp[0][2] << ", " << tmp[0][3] << std::endl;
//  std::cout << tmp[1][0] << ", " << tmp[1][1] << ", " << tmp[1][2] << ", " << tmp[1][3] << std::endl;
//  std::cout << tmp[2][0] << ", " << tmp[2][1] << ", " << tmp[2][2] << ", " << tmp[2][3] << std::endl;
//  std::cout << tmp[3][0] << ", " << tmp[3][1] << ", " << tmp[3][2] << ", " << tmp[3][3] << std::endl;
}  // namespace dev