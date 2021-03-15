#include <fstream>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "imgui.h"

// #include "portable-file-dialogs.h"
// #include "nlohmann/json.hpp"

#include "glk/simple_lines.hpp"
#include "glk/glsl_shader.hpp"

#include "dev/laser.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "dev/util.hpp"

#include "calibration/task_back_ground.hpp"
#include "calibration/two_lasers_calib.hpp"

#include "algorithm/line.h"
// #include "algorithm/util.h"

// ---- 相机-单线激光标定状态
// 空闲
#define STATE_IDLE 0
// 启动
#define STATE_START 1
// 获取一对儿打在标定墙上的激光数据
#define STATE_GET_VALID_PNTS 2
// 检查数据稳定性,连续5帧姿态没有大的变化，取中间帧作为候选
#define STATE_CHECK_STEADY 3
// 在标定过程中
#define STATE_IN_CALIB 4

namespace calibration {

TwoLasersCalib::TwoLasersCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr)
    : BaseCalib(sensor_manager_ptr) {
  // 初始化显示设备
  for (auto& laser_inst : laser_insts_) {
    laser_inst.img_show_dev_ptr = std::make_shared<dev::ImageShow>();
  }

  // 设置id
  laser_insts_[0].id = 0;
  laser_insts_[1].id = 1;

  // 后台任务
  task_ptr_ = std::make_shared<calibration::Task>();
}

void TwoLasersCalib::draw_gl(glk::GLSLShader& shader) {

  if (!laser_insts_[0].laser_lines_drawable_ptr) {
    return;
  }

  shader.set_uniform("color_mode", 2);

  for (const auto& laser_inst : laser_insts_) {
    shader.set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());
    laser_inst.laser_lines_drawable_ptr->draw(shader);
  }
}

void TwoLasersCalib::update_show() {
  // 3d空间直线信息
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;
  std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>> infos;

  const double y_start = -2.;
  const double y_end = 2.;
  double x_start, x_end;

  for (auto& laser_inst : laser_insts_) {
    // 更新显示图象
    laser_inst.img_show_dev_ptr->update_image(laser_inst.img_ptr);

    // 计算3d空间直线顶点坐标
    vertices.clear();
    colors.clear();
    infos.clear();

    for (const auto& line_params : laser_inst.laser_lines_params) {
      x_start = -(line_params(1) * y_start + line_params(2)) / line_params(0);
      x_end = -(line_params(1) * y_end + line_params(2)) / line_params(0);

      // 添加直线顶点
      vertices.emplace_back(x_start, y_start, 0);
      vertices.emplace_back(x_end, y_end, 0);

      // 添加颜色信息
      if (laser_inst.id == 0) {
        colors.emplace_back(0.f, 255.f, 255.f, 1.0f);
        colors.emplace_back(0.f, 255.f, 255.f, 1.0f);
      } else {
        colors.emplace_back(255.f, 0.f, 255.f, 1.0f);
        colors.emplace_back(255.f, 0.f, 255.f, 1.0f);
      }

      // 添加额外信息
      infos.emplace_back(laser_inst.laser_dev_ptr->sensor_id, laser_inst.id, 1, 0);
      infos.emplace_back(laser_inst.laser_dev_ptr->sensor_id, laser_inst.id, 2, 0);
    }

    // 重置3d直线
    laser_inst.laser_lines_drawable_ptr.reset(new glk::SimpleLines(vertices, colors, infos));
    // std::cout << "laser_lines_drawable_ptr: " << laser_inst.laser_lines_drawable_ptr << std::endl;
  }
}

void TwoLasersCalib::update_data() {
  std::lock_guard<std::mutex> lock(mtx_);

  // 更新数据
  for (auto& laser_inst : laser_insts_) {
    if (!laser_inst.laser_data_ptr) {
      laser_inst.laser_data_ptr = laser_inst.laser_dev_ptr->data();
      if (laser_inst.laser_data_ptr) {
        laser_inst.is_new_data = true;
      }
    } else {
      auto laser = laser_inst.laser_dev_ptr->data();
      if (laser_inst.laser_data_ptr->header.stamp.nsec != laser->header.stamp.nsec) {
        laser_inst.laser_data_ptr = laser;
        laser_inst.is_new_data = true;
      }
    }
  }
}

bool TwoLasersCalib::get_valid_points() {
  // 将激光锁定
  {
    std::lock_guard<std::mutex> lock(mtx_);
    laser_insts_[0].calib_data_ptr = laser_insts_[0].laser_data_ptr;
    laser_insts_[1].calib_data_ptr = laser_insts_[1].laser_data_ptr;
  }

  // 检测结果
  bool res = true;
  // 检测激光中的直线
  for (auto& laser_inst : laser_insts_) {
    auto start_time = ros::WallTime::now();
    cv::Mat laser_img_show;
    algorithm::Line line(*laser_inst.calib_data_ptr, 180.0, 4.0);
    if (line.find_line_ransac(laser_inst.laser_lines_params, laser_img_show)) {
      auto end_time = ros::WallTime::now();
      double time_used = (end_time - start_time).toSec() * 1000;

      // 显示用时
      cv::putText(laser_img_show, "time used (ms):  " + std::to_string(time_used), cv::Point2f(10, 15),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));
      // 显示图例
      cv::putText(laser_img_show, "RANSAC -", cv::Point2f(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.3,
                  cv::Scalar(150, 0, 0));
      cv::putText(laser_img_show, "FITTING -", cv::Point2f(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.3,
                  cv::Scalar(0, 0, 150));

      // 保存图象数据
      laser_inst.img_ptr.reset(new const cv_bridge::CvImage(laser_inst.calib_data_ptr->header, "rgb8", laser_img_show));
    } else {
      res = false;
    }
  }

  return res;
}

void TwoLasersCalib::calibration() {
  // 首先获取最新的数据
  update_data();

  switch (cur_state_) {
    case STATE_IDLE:
      cur_state_ = next_state_;
      break;
    case STATE_START:
      // 都有新数据才计算
      if (laser_insts_[0].is_new_data && laser_insts_[1].is_new_data) {
        laser_insts_[0].is_new_data = false;
        laser_insts_[1].is_new_data = false;
        cur_state_ = STATE_GET_VALID_PNTS;
      } else {
        cur_state_ = STATE_IDLE;
      }
      break;
    case STATE_GET_VALID_PNTS:
      // 执行任务
      if (task_ptr_->do_task("get_pose_and_points", std::bind(&TwoLasersCalib::get_valid_points, this))) {
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          update_show();

          // cur_state_ = STATE_CHECK_STEADY;
          cur_state_ = STATE_IDLE;
        } else {
          cur_state_ = STATE_IDLE;
        }
      }
      break;
  }
}

void TwoLasersCalib::draw_ui() {
  if (!is_show_window_) {
    return;
  }
  // 新建窗口
  ImGui::Begin("Two Lasers Calibration", &is_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  // 激光选择
  draw_sensor_selector<dev::Laser>("laser 1", dev::LASER, laser_insts_[0].laser_dev_ptr);
  // 激光选择
  draw_sensor_selector<dev::Laser>("laser 2", dev::LASER, laser_insts_[1].laser_dev_ptr);

  // 设备就绪后才能标定
  if (laser_insts_[0].laser_dev_ptr && laser_insts_[1].laser_dev_ptr) {
    ImGui::SameLine();
    // 选择是否显示图像
    if (ImGui::Checkbox("show image", &is_show_image_)) {
      if (is_show_image_) {
        laser_insts_[0].img_show_dev_ptr->enable("ts calib laser 1", false);
        laser_insts_[1].img_show_dev_ptr->enable("ts calib laser 2", false);
      } else {
        laser_insts_[0].img_show_dev_ptr->disable();
        laser_insts_[1].img_show_dev_ptr->disable();
      }
    }

    // 标定逻辑
    calibration();

    if (next_state_ == STATE_IDLE) {
      if (ImGui::Button("start")) {
        // 两个设备名称不能一样
        if (laser_insts_[0].laser_dev_ptr->sensor_id == laser_insts_[1].laser_dev_ptr->sensor_id) {
          std::string msg = "two lasers are the same!";
          dev::show_pfd_info("two lasers calibration", msg);
        } else {
          next_state_ = STATE_START;
        }
      }
    } else {
      if (ImGui::Button("stop")) {
        next_state_ = STATE_IDLE;
      }
    }

    // 标定状态只需要设定一次
    if (cur_state_ == STATE_IN_CALIB) {
      next_state_ = STATE_IDLE;
    }
  }

  ImGui::End();

  // 显示图像
  if (is_show_image_) {
    laser_insts_[0].img_show_dev_ptr->show_image(is_show_image_);
    laser_insts_[1].img_show_dev_ptr->show_image(is_show_image_);
  }
}

}  // namespace calibration
