#include <iostream>
#include <ros/master.h>

#include "imgui.h"
#include "portable-file-dialogs.h"
#include "dev/camera.hpp"
#include "dev/sensor_data.hpp"
#include "camera_model/camera_models/CameraFactory.h"

namespace dev {

Camera::Camera(const std::string& name, ros::NodeHandle& ros_nh) : Sensor(name, ros_nh, SENSOR_TYPE::CAMERA, "CAMERA") {
  topic_list_.resize(2);
  // 设置图像数据接收
  image_data_ = std::make_shared<SensorData<sensor_msgs::Image>>(nh_, 10);
  // 频率/2
  image_data_->set_data_rate(2);

  // 设置深度点云数据接收
  points_data_ = std::make_shared<SensorData<sensor_msgs::PointCloud2>>(nh_, 10);
  // 频率/2
  points_data_->set_data_rate(2);
}

void Camera::draw_gl(glk::GLSLShader& shader) {}

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

static void HelpMarker(const char* desc) {
  ImGui::TextDisabled("(?)");
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextUnformatted(desc);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void Camera::draw_ui_parms() {
  static double const_0 = 0.0;

  ImGui::BeginGroup();
  ImGui::Text("width:%d", inst_ptr_->imageWidth());
  ImGui::SameLine();
  ImGui::Text("height:%d", inst_ptr_->imageHeight());
  ImGui::SameLine();
  HelpMarker("image size will be modified \nafter received image topic.");

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
  ImGui::EndGroup();

  // 及时更新修改到相机参数
  inst_ptr_->readParameters(inst_params_);
}

static void get_topic_name_from_list(const std::string& target_topic_type, std::vector<std::string>& candidates) {
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  candidates.clear();

  for (auto& info : master_topics) {
    if (info.datatype == target_topic_type) {
      candidates.push_back(info.name);
    }
  }
}

void Camera::draw_ui_topic_name() {
  // 名称控件变量
  static char image_topic_name_char[128]{""};
  static char points_topic_name_char[128]{""};
  static std::vector<std::string> topic_name_list;

  ImVec2 size = ImGui::GetItemRectSize();
  // ros话题
  ImGui::BeginGroup();
  ImGui::Separator();

  // ---- 修改image topic名称
  if (ImGui::Checkbox("##image_topic", &enable_topic_[0])){
    // 如果topic有效启用数据接收
    if (topic_list_[0].empty()){
      pfd::message message("warning", "please set topic name first!", pfd::choice::ok);
      while (!message.ready()) {
        usleep(1000);
      }
    } else {
      image_data_->subscribe(topic_list_[0], 5);
    }
  }
  // 提示设备状态
  if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("pick to enable image receive");
  }
  ImGui::SameLine();
  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();
  ImGui::Text("image topic name:");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(size.x * 0.6f);
  // 只有按回车才保存
  if (ImGui::InputText("##image topic name", image_topic_name_char, IM_ARRAYSIZE(image_topic_name_char),
                       ImGuiInputTextFlags_AutoSelectAll | ImGuiInputTextFlags_EnterReturnsTrue)) {
    if (image_topic_name_char[0] != '\0' && image_topic_name_char[0] != ' ') {
      topic_list_[0] = image_topic_name_char;
    }
  } else {
    // 恢复名称
    memset(image_topic_name_char, 0, 128);
    memcpy(image_topic_name_char, topic_list_[0].c_str(), topic_list_[0].length());
  }
  ImGui::SameLine();
  // 从ros系统中选择话题
  int selected_id = -1;
  if (ImGui::Button("...##image")) {
    get_topic_name_from_list("sensor_msgs/Image", topic_name_list);
    ImGui::OpenPopup("popup##image");
  }
  if (ImGui::BeginPopup("popup##image")) {
    if (topic_name_list.empty()) {
      ImGui::Text("no sensor_msgs/Image msgs available.");
    } else {
      ImGui::Text("[sensor_msgs/Image list]");
      ImGui::Separator();
      for (int i = 0; i < topic_name_list.size(); i++) {
        // 按下
        if (ImGui::Selectable(topic_name_list[i].c_str())) {
          selected_id = i;
        }
      }
    }
    // 变更话题名称
    if (selected_id != -1) {
      topic_list_[0] = topic_name_list[selected_id];
    }

    ImGui::EndPopup();
  }

  // ---- 修改depth points topic名称
  ImGui::Checkbox("##depth_topic", &enable_topic_[1]);
  // 提示设备状态
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("pick to enable points receive");
  }
  ImGui::SameLine();
  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();
  ImGui::Text("depth points topic:");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.86f);
  // 只有按回车才保存
  if (ImGui::InputText("##depth points topic", points_topic_name_char, IM_ARRAYSIZE(points_topic_name_char),
                       ImGuiInputTextFlags_AutoSelectAll | ImGuiInputTextFlags_EnterReturnsTrue)) {
    if (points_topic_name_char[0] != '\0' && points_topic_name_char[0] != ' ') {
      topic_list_[1] = points_topic_name_char;
    }
  } else {
    // 恢复名称
    memset(points_topic_name_char, 0, 128);
    memcpy(points_topic_name_char, topic_list_[1].c_str(), topic_list_[1].length());
  }
  ImGui::SameLine();
  // 从ros系统中选择话题
  selected_id = -1;
  if (ImGui::Button("...##depth")) {
    get_topic_name_from_list("sensor_msgs/PointCloud2", topic_name_list);
    ImGui::OpenPopup("popup##depth");
  }
  if (ImGui::BeginPopup("popup##depth")) {
    if (topic_name_list.empty()) {
      ImGui::Text("no sensor_msgs/Image msgs available.");
    } else {
      ImGui::Text("[sensor_msgs/PointCloud2 list]");
      ImGui::Separator();
      for (int i = 0; i < topic_name_list.size(); i++) {
        // 按下
        if (ImGui::Selectable(topic_name_list[i].c_str())) {
          selected_id = i;
        }
      }
    }
    // 变更话题名称
    if (selected_id != -1) {
      topic_list_[1] = topic_name_list[selected_id];
    }

    ImGui::EndPopup();
  }
  ImGui::EndGroup();
}

void Camera::draw_ui() {
  // 名称控件变量
  static char name_char[128]{" "};
  // 相机类型控件变量
  const char* camera_type[] = {"KANNALA_BRANDT", "MEI", "PINHOLE"};
  static int current_camera_type = 2;

  // ------ test
  static std::string default_path{"/home/anson/catkin_map/src/robot_basic_tools/config/camera_config"};

  if (!is_show_window_) {
    return;
  }

  // 初始化为设备名称
  if (name_char[0] == ' ') {
    memcpy(name_char, sensor_name.c_str(), sensor_name.length());
  }

  // 保证窗口名称唯一
  ImGui::SetNextWindowPos(ImVec2(300, 22), ImGuiCond_FirstUseEver);

  std::string win_name = sensor_name + "##" + std::to_string(sensor_id);
  ImGui::Begin(win_name.c_str(), &is_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::BeginGroup();

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
      inst_ptr_->cameraName() = sensor_name;
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
    std::unique_ptr<pfd::open_file> dialog(new pfd::open_file("choose file", default_path, filters));
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
        std::cout << "load ok!" << sensor_name << ": type [" << inst_ptr_->modelType()
                  << "], params size: " << inst_params_.size() << std::endl;
      } else {
        std::string msg = "load camera from [" + file_paths[0] + "] failed!";
        pfd::message message("load from yaml", msg, pfd::choice::ok);
        while (!message.ready()) {
          usleep(1000);
        }
      }
    }
  }
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("load from .yaml");
  }

  // 将相机参数写入文件
  if (inst_ptr_) {
    ImGui::SameLine();
    if (ImGui::Button("W")) {
      // 选择保存文件路径
      std::vector<std::string> filters = {"camera config file (.yaml)", "*.yaml"};
      std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file", default_path, filters));
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
      ImGui::SetTooltip("save to .yaml");
    }
  }

  // 相机类型控制
  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();
  ImGui::Text("camera type:");
  ImGui::SameLine();

  if (ImGui::Combo("##camera type", &current_camera_type, camera_type, IM_ARRAYSIZE(camera_type))) {
    // 如果状态改变 询问改变
    if (inst_ptr_) {
      std::string msg = "are you sure to change camera type ?";
      pfd::message message("type change", msg, pfd::choice::ok_cancel);
      while (!message.ready()) {
        usleep(1000);
      }
      // 新建实例
      if (message.result() == pfd::button::ok) {
        creat_instance(current_camera_type);
      } else {
        current_camera_type = inst_ptr_->modelType();
      }
    }
  }

  // 如果没有camera对象，可以选择新建
  if (!inst_ptr_) {
    ImGui::SameLine();
    if (ImGui::Button("new")) {
      creat_instance(current_camera_type);
    }
    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("create a new camera");
    }
    ImGui::EndGroup();
  } else {
    ImGui::EndGroup();
    // 展示当前相机参数
    draw_ui_parms();
    // 选择相机话题
    draw_ui_topic_name();
  }

  ImGui::End();
}

}  // namespace dev