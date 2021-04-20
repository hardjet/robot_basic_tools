#include <iostream>

#include "imgui.h"
#include "portable-file-dialogs.h"
#include <cv_bridge/cv_bridge.h>

#include "glk/glsl_shader.hpp"
#include "glk/primitives/primitives.hpp"

#include "dev/camera.hpp"
#include "dev/sensor_data.hpp"
#include "dev/image_show.hpp"
#include "dev/util.hpp"
#include "camera_model/camera_models/CameraFactory.h"

namespace dev {

Camera::Camera(const std::string& name, ros::NodeHandle& ros_nh) : Sensor(name, ros_nh, SENSOR_TYPE::CAMERA) {
  sensor_topic_list_.resize(2);
  // 设置图像数据接收
  image_data_ptr_ = std::make_shared<SensorData<sensor_msgs::Image>>(nh_, 10);
  // 频率/2
  // image_data_ptr_->set_data_rate(2);

  // 设置深度点云数据接收
  points_data_ptr_ = std::make_shared<SensorData<sensor_msgs::PointCloud2>>(nh_, 10);
  // 频率/2
  // points_data_ptr_->set_data_rate(2);

  // 图像显示
  im_show_ptr_ = std::make_shared<dev::ImageShow>();
}

boost::shared_ptr<cv_bridge::CvImage const> Camera::data() { return image_cv_ptr_; }

void Camera::draw_gl(glk::GLSLShader& shader) {
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.rotate(T_.block<3, 3>(0, 0));
  T.pretranslate(T_.block<3, 1>(0, 3));

  // 画坐标系
  shader.set_uniform("color_mode", 2);
  shader.set_uniform("model_matrix", (T * Eigen::UniformScaling<float>(0.05f)).matrix());
  const auto& coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
  coord.draw(shader);

  if (ply_model_ptr_) {
    shader.set_uniform("color_mode", 1);
    // shader.set_uniform("model_matrix",
    //                    (Eigen::Translation3f(Eigen::Vector3f{1.0, 1.0, 1.0}) *
    //                    Eigen::Isometry3f::Identity()).matrix());
    shader.set_uniform("model_matrix", T_);
    shader.set_uniform("material_color", Eigen::Vector4f(data_color_[0], data_color_[1], data_color_[2], 1.0f));
    ply_model_ptr_->draw(shader);
  }
}

bool Camera::cv_convert(boost::shared_ptr<const sensor_msgs::Image>& msg) {
  bool res = true;
  // 尝试转换为BGR
  try {
    // image原本发送的就是'rgb8'格式，但是imshow显示使用BGR8格式
    // 原始msg中的图像就是rgb8格式，这样转换最快
    image_cv_ptr_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    // 尝试转换为灰度图
    try {
      image_cv_ptr_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
      res = false;
    }
  }
  return res;
}

void Camera::check_online_status() {
  // 在线状态
  bool online = false;

  // 获取图像最新数据
  auto image_msg_ptr = image_data_ptr_->data();
  // 如果有数据
  if (image_msg_ptr) {
    // 设置在线状态
    if (ros::Time::now().sec - image_msg_ptr->header.stamp.sec < 2) {
      online = true;
    }

    // 检查图像是否需要更新，避免重复更新
    if (!image_cv_ptr_ || image_cv_ptr_->header.stamp.nsec != image_msg_ptr->header.stamp.nsec) {
      if (cv_convert(image_msg_ptr)) {
        // 更新图像数据
        im_show_ptr_->update_image(image_cv_ptr_);
      }
    }
  }

  // 获取深度点云最新数据
  auto depth_data_ptr = points_data_ptr_->data();
  if (depth_data_ptr) {
    // 设置在线状态
    if (ros::Time::now().sec - depth_data_ptr->header.stamp.sec < 2) {
      online = true;
    }
  }

  is_online_ = online;
}

void Camera::creat_instance(int current_camera_type) {
  inst_ptr_ = nullptr;
  switch (current_camera_type) {
    case camera_model::Camera::ModelType::KANNALA_BRANDT:
      inst_ptr_ = camera_model::CameraFactory::instance()->generateCamera(
          camera_model::Camera::ModelType::KANNALA_BRANDT, sensor_name, cv::Size{640, 480});
      break;
    case camera_model::Camera::ModelType::MEI:
      inst_ptr_ = camera_model::CameraFactory::instance()->generateCamera(camera_model::Camera::ModelType::MEI,
                                                                          sensor_name, cv::Size{640, 480});
      break;
    case camera_model::Camera::ModelType::PINHOLE:
      inst_ptr_ = camera_model::CameraFactory::instance()->generateCamera(camera_model::Camera::ModelType::PINHOLE,
                                                                          sensor_name, cv::Size{640, 480});
      break;
    default:
      break;
  }
  inst_ptr_->writeParameters(inst_params_);
}

void Camera::update_params() {
  // 更新ui参数
  inst_ptr_->writeParameters(inst_params_);
}

void Camera::draw_ui_params() {
  static double const_0 = 0.0;

  // ImGui::BeginGroup();
  ImGui::Text("width:%d", inst_ptr_->imageWidth());
  ImGui::SameLine();
  ImGui::Text("height:%d", inst_ptr_->imageHeight());
  ImGui::SameLine();
  help_marker("image size will be modified \nafter received image msgs.");

  ImGui::Separator();
  ImGui::Text("params:");

  // 设定宽度
  ImGui::PushItemWidth(80);
  switch (inst_ptr_->modelType()) {
    case camera_model::Camera::ModelType::KANNALA_BRANDT:
      ImGui::DragScalar("mu", ImGuiDataType_Double, &inst_params_[4], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("mv", ImGuiDataType_Double, &inst_params_[5], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("u0", ImGuiDataType_Double, &inst_params_[6], 1.0, nullptr, nullptr, "%.2f");
      ImGui::SameLine();
      ImGui::DragScalar("v0", ImGuiDataType_Double, &inst_params_[7], 1.0, nullptr, nullptr, "%.2f");
      // 新行
      ImGui::DragScalar("k2", ImGuiDataType_Double, &inst_params_[0], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("k3", ImGuiDataType_Double, &inst_params_[1], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("k4", ImGuiDataType_Double, &inst_params_[2], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("k5", ImGuiDataType_Double, &inst_params_[3], 0.001, nullptr, nullptr, "%.6f");
      break;
    case camera_model::Camera::ModelType::MEI:
      // 新行
      ImGui::DragScalar("xi", ImGuiDataType_Double, &inst_params_[0], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("u0", ImGuiDataType_Double, &inst_params_[7], 0.001, nullptr, nullptr, "%.2f");
      ImGui::SameLine();
      ImGui::DragScalar("v0", ImGuiDataType_Double, &inst_params_[8], 0.001, nullptr, nullptr, "%.2f");
      // 新行
      ImGui::DragScalar("gamma1", ImGuiDataType_Double, &inst_params_[5], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("gamma2", ImGuiDataType_Double, &inst_params_[6], 0.001, nullptr, nullptr, "%.6f");
      // 新行
      ImGui::DragScalar("k1", ImGuiDataType_Double, &inst_params_[1], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("k2", ImGuiDataType_Double, &inst_params_[2], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("p1", ImGuiDataType_Double, &inst_params_[3], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("p2", ImGuiDataType_Double, &inst_params_[4], 0.001, nullptr, nullptr, "%.6f");
      break;
    case camera_model::Camera::ModelType::PINHOLE:
      ImGui::DragScalar("fx", ImGuiDataType_Double, &inst_params_[4], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("fy", ImGuiDataType_Double, &inst_params_[5], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("cx", ImGuiDataType_Double, &inst_params_[6], 1.0, &const_0, nullptr, "%.2f");
      ImGui::SameLine();
      ImGui::DragScalar("cy", ImGuiDataType_Double, &inst_params_[7], 1.0, &const_0, nullptr, "%.2f");
      // 新行
      ImGui::DragScalar("k1", ImGuiDataType_Double, &inst_params_[0], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("k2", ImGuiDataType_Double, &inst_params_[1], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("p1", ImGuiDataType_Double, &inst_params_[2], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("p2", ImGuiDataType_Double, &inst_params_[3], 0.001, nullptr, nullptr, "%.6f");

      break;
    default:
      break;
  }
  ImGui::PopItemWidth();
  // ImGui::EndGroup();

  // 及时更新修改到相机参数
  inst_ptr_->readParameters(inst_params_);
}

void Camera::draw_ui_topic_name() {
  // 名称控件变量
  char image_topic_name_char[128]{""};
  char points_topic_name_char[128]{""};

  memcpy(image_topic_name_char, sensor_topic_list_[0].c_str(), sensor_topic_list_[0].length());
  memcpy(points_topic_name_char, sensor_topic_list_[1].c_str(), sensor_topic_list_[1].length());

  // ImVec2 size = ImGui::GetItemRectSize();
  // ros话题
  // ImGui::BeginGroup();
  ImGui::Separator();

  // ---- 修改image topic名称
  if (ImGui::Checkbox("##image_topic", &b_enable_topic_[0])) {
    // 选中状态
    if (b_enable_topic_[0]) {
      // 如果topic有效才启用数据接收
      if (sensor_topic_list_[0].empty()) {
        show_pfd_info("warning", "please set topic name first!");
        b_enable_topic_[0] = false;
      } else {
        image_data_ptr_->subscribe(sensor_topic_list_[0], 5);
      }
    } else {
      // 暂停接收
      image_data_ptr_->unsubscribe();
    }
  }
  // 提示设备状态
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("pick to enable image receive");
  }
  ImGui::SameLine();
  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();
  ImGui::Text("image topic name  :");
  ImGui::SameLine();
  // ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.8f);
  // 只有按回车才保存
  if (ImGui::InputTextWithHint("##image_topic_name", "press 'ENTER' to save", image_topic_name_char, 128,
                               ImGuiInputTextFlags_AutoSelectAll | ImGuiInputTextFlags_EnterReturnsTrue)) {
    if (image_topic_name_char[0] != '\0' && image_topic_name_char[0] != ' ') {
      sensor_topic_list_[0] = image_topic_name_char;
      // 暂停接收
      image_data_ptr_->unsubscribe();
      b_enable_topic_[0] = false;
    }
  } else {
    // 恢复名称
    memset(image_topic_name_char, 0, 128);
    memcpy(image_topic_name_char, sensor_topic_list_[0].c_str(), sensor_topic_list_[0].length());
  }
  ImGui::SameLine();
  // 从ros系统中选择话题
  int selected_id = -1;
  if (ImGui::Button("...##image")) {
    get_topic_name_from_list("sensor_msgs/Image", ros_topic_selector_);
    ImGui::OpenPopup("popup##image");
  }
  if (ImGui::BeginPopup("popup##image")) {
    if (ros_topic_selector_.empty()) {
      ImGui::Text("no sensor_msgs/Image msgs available.");
    } else {
      ImGui::Text("[sensor_msgs/Image list]");
      ImGui::Separator();
      for (int i = 0; i < ros_topic_selector_.size(); i++) {
        // 按下
        if (ImGui::Selectable(ros_topic_selector_[i].c_str())) {
          selected_id = i;
        }
      }
    }
    // 变更话题名称
    if (selected_id != -1) {
      sensor_topic_list_[0] = ros_topic_selector_[selected_id];
      // 暂停接收
      image_data_ptr_->unsubscribe();
      b_enable_topic_[0] = false;
    }

    ImGui::EndPopup();
  }

  // ---- 修改depth points topic名称
  if (ImGui::Checkbox("##depth_topic", &b_enable_topic_[1])) {
    // 选中状态
    if (b_enable_topic_[1]) {
      // 如果topic有效才启用数据接收
      if (sensor_topic_list_[1].empty()) {
        show_pfd_info("warning", "please set topic name first!");
        b_enable_topic_[1] = false;
      } else {
        points_data_ptr_->subscribe(sensor_topic_list_[1], 5);
      }
    } else {
      // 暂停接收
      points_data_ptr_->unsubscribe();
    }
  }
  // 提示设备状态
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("pick to enable points receive");
  }
  ImGui::SameLine();
  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();
  ImGui::Text("depth points topic:");
  ImGui::SameLine();
  // ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.86f);
  // 只有按回车才保存
  if (ImGui::InputTextWithHint("##depth_topic_name", "press 'ENTER' to save", points_topic_name_char, 128,
                               ImGuiInputTextFlags_AutoSelectAll | ImGuiInputTextFlags_EnterReturnsTrue)) {
    if (points_topic_name_char[0] != '\0' && points_topic_name_char[0] != ' ') {
      sensor_topic_list_[1] = points_topic_name_char;
      // 暂停接收
      points_data_ptr_->unsubscribe();
      b_enable_topic_[1] = false;
    }
  } else {
    // 恢复名称
    memset(points_topic_name_char, 0, 128);
    memcpy(points_topic_name_char, sensor_topic_list_[1].c_str(), sensor_topic_list_[1].length());
  }
  ImGui::SameLine();
  // 从ros系统中选择话题
  selected_id = -1;
  if (ImGui::Button("...##depth")) {
    get_topic_name_from_list("sensor_msgs/PointCloud2", ros_topic_selector_);
    ImGui::OpenPopup("popup##depth");
  }
  if (ImGui::BeginPopup("popup##depth")) {
    if (ros_topic_selector_.empty()) {
      ImGui::Text("no sensor_msgs/Image msgs available.");
    } else {
      ImGui::Text("[sensor_msgs/PointCloud2 list]");
      ImGui::Separator();
      for (int i = 0; i < ros_topic_selector_.size(); i++) {
        // 按下
        if (ImGui::Selectable(ros_topic_selector_[i].c_str())) {
          selected_id = i;
        }
      }
    }
    // 变更话题名称
    if (selected_id != -1) {
      sensor_topic_list_[1] = ros_topic_selector_[selected_id];
      // 暂停接收
      points_data_ptr_->unsubscribe();
      b_enable_topic_[1] = false;
    }

    ImGui::EndPopup();
  }
  // ImGui::EndGroup();
}

void Camera::draw_ui() {
  check_online_status();

  if (!b_show_window_) {
    return;
  }

  // 名称控件变量
  char name_char[128]{""};
  // 相机类型控件变量
  const char* camera_type[] = {"KANNALA_BRANDT", "MEI", "PINHOLE"};

  // 初始化为设备名称
  memcpy(name_char, sensor_name.c_str(), sensor_name.length());

  // ImGui::SetNextWindowPos(ImVec2(300, 22), ImGuiCond_FirstUseEver);
  // 保证窗口名称唯一
  std::string win_name = sensor_name + "##" + std::to_string(sensor_id);
  ImGui::Begin(win_name.c_str(), &b_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  // ImGui::BeginGroup();

  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();

  // 修改名称
  ImGui::Text("sensor name:");
  ImGui::SameLine();
  // 只有按回车才保存
  if (ImGui::InputTextWithHint("##sensor name", "press 'ENTER' to save", name_char, IM_ARRAYSIZE(name_char),
                               ImGuiInputTextFlags_AutoSelectAll | ImGuiInputTextFlags_EnterReturnsTrue)) {
    if (name_char[0] != '\0' && name_char[0] != ' ') {
      sensor_name = name_char;
      if (inst_ptr_) {
        inst_ptr_->cameraName() = sensor_name;
      }
    }
  } else {
    // 恢复名称
    memset(name_char, 0, 128);
    memcpy(name_char, sensor_name.c_str(), sensor_name.length());
  }

  ImGui::SameLine();
  // 从文件加载相机
  if (ImGui::Button("R")) {
    // 选择加载文件路径
    std::vector<std::string> filters = {"camera config file (.yaml)", "*.yaml"};
    std::unique_ptr<pfd::open_file> dialog(new pfd::open_file("choose file", config_default_path, filters));
    while (!dialog->ready()) {
      usleep(1000);
    }

    auto file_paths = dialog->result();
    if (!file_paths.empty()) {
      // 从文件加载相机
      auto cam_ptr = camera_model::CameraFactory::instance()->generateCameraFromYamlFile(file_paths[0]);
      if (cam_ptr && cam_ptr->modelType() <= camera_model::Camera::ModelType::PINHOLE) {
        inst_ptr_ = cam_ptr;
        // 更新名称
        sensor_name = inst_ptr_->cameraName();
        // 加载参数
        inst_ptr_->writeParameters(inst_params_);
        // 更新相机类型
        current_camera_type_ = inst_ptr_->modelType();
        std::cout << "load ok!" << sensor_name << ": type [" << inst_ptr_->modelType()
                  << "], params size: " << inst_params_.size() << std::endl;
      } else {
        std::string msg = "load camera from [" + file_paths[0] + "] failed!";
        show_pfd_info("load from yaml", msg);
      }
    }
  }
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("load camera from .yaml");
  }

  // 将相机参数写入文件
  if (inst_ptr_) {
    ImGui::SameLine();
    if (ImGui::Button("W")) {
      // 选择保存文件路径
      std::vector<std::string> filters = {"camera config file (.yaml)", "*.yaml"};
      std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file", config_default_path, filters));
      while (!dialog->ready()) {
        usleep(1000);
      }

      auto file_path = dialog->result();
      if (!file_path.empty()) {
        // 导出参数
        inst_ptr_->writeParametersToYamlFile(file_path);
      }
    }
    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("save camera to .yaml");
    }

    // 加载3d模型
    ImGui::SameLine();
    if (ImGui::Button("3D")) {
      load_model();
    }
    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("load 3d model from .ply file");
    }

    // 选择数据颜色
    ImGui::SameLine();
    draw_data_color_selector();
  }

  // 相机类型控制
  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();
  ImGui::Text("camera type:");
  ImGui::SameLine();

  if (ImGui::Combo("##camera type", &current_camera_type_, camera_type, IM_ARRAYSIZE(camera_type))) {
    // 如果状态改变 询问改变
    if (inst_ptr_) {
      std::string msg = "are you sure to change camera type ?";
      pfd::message message("type change", msg, pfd::choice::ok_cancel);
      while (!message.ready()) {
        usleep(1000);
      }
      // 新建实例
      if (message.result() == pfd::button::ok) {
        creat_instance(current_camera_type_);
      } else {
        current_camera_type_ = inst_ptr_->modelType();
      }
    }
  }

  // 如果没有camera对象，可以选择新建
  if (!inst_ptr_) {
    ImGui::SameLine();
    if (ImGui::Button("new")) {
      creat_instance(current_camera_type_);
    }
    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("create a new camera");
    }
    // ImGui::EndGroup();
  } else {
    ImGui::SameLine();
    // 选择是否显示图像
    if (ImGui::Checkbox("show image", &b_show_image_)) {
      if (b_show_image_) {
        im_show_ptr_->enable(sensor_name, false);
      } else {
        im_show_ptr_->disable();
      }
    }
    // ImGui::EndGroup();
    // 展示当前相机参数
    draw_ui_params();
    // 选择相机话题
    draw_ui_topic_name();
  }

  ImGui::End();

  // 检查设备在线状态
  check_online_status();

  im_show_ptr_->show_image(b_show_image_);
}

}  // namespace dev