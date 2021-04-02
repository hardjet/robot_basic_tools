#include <memory>
#include "imgui.h"

#include "glk/glsl_shader.hpp"
#include "glk/pointcloud_buffer.hpp"

#include "dev/sensor_data.hpp"
#include "dev/laser.hpp"
#include "dev/util.hpp"

namespace dev {

Laser::Laser(const std::string& name, ros::NodeHandle& ros_nh) : Sensor(name, ros_nh, SENSOR_TYPE::LASER) {
  sensor_topic_list_.resize(1);

  // 设置深度点云数据接收
  laser_data_ptr_ = std::make_shared<SensorData<sensor_msgs::LaserScan>>(nh_, 5);
  // 频率/2
  laser_data_ptr_->set_data_rate(2);

  // 设置默认颜色
  data_color_[0] = 0.;
  data_color_[1] = 1.;
  data_color_[2] = 0.;
}

boost::shared_ptr<const sensor_msgs::LaserScan> Laser::data() { return laser_data_ptr_->data(); }

static pcl::PointCloud<pcl::PointXYZI>::Ptr convert_laserscan_to_pc(sensor_msgs::LaserScan::ConstPtr& laser_scan) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());

  float angle_min = laser_scan->angle_min;
  float angle_increment = laser_scan->angle_increment;

  // 填充点云
  // 当前range数据对应的角度
  float cur_angle;
  for (size_t i = 0; i < laser_scan->ranges.size(); i++) {
    cur_angle = angle_min + i * angle_increment;
    pcl::PointXYZI p{};
    p.x = std::cos(cur_angle) * laser_scan->ranges.at(i);
    p.y = std::sin(cur_angle) * laser_scan->ranges.at(i);
    p.z = 0.;
    p.intensity = laser_scan->intensities.empty() ? float(0.) : laser_scan->intensities.at(i);
    pc->push_back(p);
  }

  return pc;
}

void Laser::free() {
  if (pointcloud_buffer_ptr_) {
    pointcloud_buffer_ptr_->free();
  }
  free_model();
}

void Laser::draw_gl(glk::GLSLShader& shader) {
  if (!b_show_laser_) {
    return;
  }

  // 获取最新的激光数据
  auto laser_data = data();

  // 如果有数据
  if (laser_data) {
    // 时间戳不等才更新
    if (laser_data->header.stamp.nsec != time_ns_) {
      pointcloud_buffer_ptr_.reset(new glk::PointCloudBuffer(convert_laserscan_to_pc(laser_data)));
      time_ns_ = laser_data->header.stamp.nsec;
    }

    if (pointcloud_buffer_ptr_) {
      // 画图
      shader.set_uniform("color_mode", 1);
      shader.set_uniform("model_matrix", T_);
      // shader.set_uniform("info_values", Eigen::Vector4i(1, 0, 0, 0));
      shader.set_uniform("material_color", Eigen::Vector4f(data_color_[0], data_color_[1], data_color_[2], 1.0f));
      pointcloud_buffer_ptr_->draw(shader);
    }
  }

  if (ply_model_ptr_) {
    shader.set_uniform("color_mode", 1);
    shader.set_uniform("model_matrix", T_);
    shader.set_uniform("material_color", Eigen::Vector4f(data_color_[0], data_color_[1], data_color_[2], 1.0f));
    ply_model_ptr_->draw(shader);
  }
}

void Laser::check_online_status() {
  bool online = false;

  // 获取深度点云最新数据
  auto laser_data_ptr = laser_data_ptr_->data();
  if (laser_data_ptr) {
    if (ros::Time::now().sec - laser_data_ptr->header.stamp.sec < 2) {
      online = true;
    }
  }

  is_online_ = online;
}

void Laser::draw_ui_topic_name() {
  // 名称控件变量
  char laser_topic_name_char[128]{""};

  memcpy(laser_topic_name_char, sensor_topic_list_[0].c_str(), sensor_topic_list_[0].length());

  ImGui::Separator();

  if (ImGui::Checkbox("##laser_topic", &b_enable_topic_)) {
    // 选中状态
    if (b_enable_topic_) {
      // 如果topic有效才启用数据接收
      if (sensor_topic_list_[0].empty()) {
        show_pfd_info("warning", "please set topic name first!");
        b_enable_topic_ = false;
      } else {
        laser_data_ptr_->subscribe(sensor_topic_list_[0], 5);
      }
    } else {
      // 暂停接收
      laser_data_ptr_->unsubscribe();
    }
  }
  // 提示设备状态
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("pick to enable laser receive");
  }
  ImGui::SameLine();
  // 保证控件中文字对齐
  ImGui::AlignTextToFramePadding();
  ImGui::Text("laser topic:");
  ImGui::SameLine();
  // ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.86f);
  // 只有按回车才保存
  if (ImGui::InputTextWithHint("##laser_topic_name", "press 'ENTER' to save", laser_topic_name_char, 128,
                               ImGuiInputTextFlags_AutoSelectAll | ImGuiInputTextFlags_EnterReturnsTrue)) {
    if (laser_topic_name_char[0] != '\0' && laser_topic_name_char[0] != ' ') {
      sensor_topic_list_[0] = laser_topic_name_char;
      // 暂停接收
      laser_data_ptr_->unsubscribe();
      b_enable_topic_ = false;
    }
  } else {
    // 恢复名称
    memset(laser_topic_name_char, 0, 128);
    memcpy(laser_topic_name_char, sensor_topic_list_[0].c_str(), sensor_topic_list_[0].length());
  }
  ImGui::SameLine();
  // 从ros系统中选择话题
  int selected_id = -1;
  if (ImGui::Button("...##laser")) {
    get_topic_name_from_list("sensor_msgs/LaserScan", ros_topic_selector_);
    ImGui::OpenPopup("popup##laser");
  }
  if (ImGui::BeginPopup("popup##laser")) {
    if (ros_topic_selector_.empty()) {
      ImGui::Text("no sensor_msgs/LaserScan msgs available.");
    } else {
      ImGui::Text("[sensor_msgs/LaserScan list]");
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
      laser_data_ptr_->unsubscribe();
      b_enable_topic_ = false;
    }

    ImGui::EndPopup();
  }
}

void Laser::draw_ui() {
  // 名称控件变量
  char name_char[128]{""};

  // 检查设备在线状态
  check_online_status();

  if (!b_show_window_) {
    return;
  }

  // 初始化为设备名称
  memcpy(name_char, sensor_name.c_str(), sensor_name.length());

  // 保证窗口名称唯一
  std::string win_name = sensor_name + "##" + std::to_string(sensor_id);
  ImGui::Begin(win_name.c_str(), &b_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

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
    }
  } else {
    // 恢复名称
    memset(name_char, 0, 128);
    memcpy(name_char, sensor_name.c_str(), sensor_name.length());
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

  // 使能数据显示
  ImGui::SameLine();
  ImGui::Checkbox("##show_laser", &b_show_laser_);
  // 提示设备状态
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("enable/disable show laser data");
  }

  //选择数据topic
  draw_ui_topic_name();

  ImGui::End();
}

}  // namespace dev