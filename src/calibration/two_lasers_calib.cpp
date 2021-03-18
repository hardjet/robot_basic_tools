#include <fstream>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "imgui.h"

#include "portable-file-dialogs.h"
#include "nlohmann/json.hpp"

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
#define STATE_GET_VALID_LINES 2
// 检查数据稳定性,连续5帧姿态没有大的变化，取中间帧作为候选
#define STATE_CHECK_STEADY 3
// 开始标定
#define STATE_START_CALIB 4
// 在标定过程中
#define STATE_IN_CALIB 5

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

  double x_start, x_end;
  double y_start, y_end;

  // 设置标定数据
  // 时间
  calib_data_.timestamp = laser_insts_[0].calib_data_ptr->header.stamp.toSec();

  for (auto& laser_inst : laser_insts_) {
    // 更新显示图象
    laser_inst.img_show_dev_ptr->update_image(laser_inst.img_ptr);

    // 计算3d空间直线顶点坐标
    vertices.clear();
    colors.clear();
    infos.clear();

    // 外扩的长度
    const double extend_len = 0.1;
    // 直线与x轴的夹角
    double theta, delta_len;

    // 设置直线参数
    calib_data_.lines_params[laser_inst.id] = laser_inst.laser_lines_params;

    for (int i = 0; i < 2; i++) {
      const auto& line_params = laser_inst.laser_lines_params[i];
      const auto& line_min_max = laser_inst.lines_min_max[i];

      // 计算角度
      theta = atan(line_params(0) / line_params(1));
      delta_len = abs(extend_len * sin(theta));
      // printf("theta: %f, delta_len: %f\n", theta * 180. / M_PI, delta_len);

      // 外扩一定长度
      y_start = line_min_max(0) - delta_len;
      y_end = line_min_max(1) + delta_len;

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

      // 计算直线上的中点
      double mid_y = (line_min_max(0) + line_min_max(1)) / 2.;
      double mid_x = -(line_params(1) * mid_y + line_params(2)) / line_params(0);
      calib_data_.mid_pts_on_line[laser_inst.id][i] = Eigen::Vector2d{mid_x, mid_y};
    }

    // 重置3d直线
    laser_inst.laser_lines_drawable_ptr.reset(new glk::SimpleLines(vertices, colors, infos));
    // std::cout << "laser_lines_drawable_ptr: " << laser_inst.laser_lines_drawable_ptr << std::endl;
  }

  calib_data_.angle = atan(calib_data_.lines_params[0][0](0) / calib_data_.lines_params[0][0](1));
  calib_data_.angle *= 180. / M_PI;
}

void TwoLasersCalib::update() {
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

bool TwoLasersCalib::get_valid_lines() {
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
    if (line.find_two_lines(laser_inst.laser_lines_params, laser_inst.lines_min_max, laser_img_show)) {
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

void TwoLasersCalib::check_and_save() {
  bool is_need_to_save = true;
  // 逐个检测角度值与第3帧比较
  for (auto& data : calib_valid_data_vec_) {
    double theta = abs(data.angle - calib_data_vec_.at(2).angle);
    // 保证间隔5°以上
    if (theta < 5.0) {
      is_need_to_save = false;
      break;
    }
  }

  if (is_need_to_save) {
    // 取第3帧保存
    calib_valid_data_vec_.push_back(calib_data_vec_.at(2));
    printf("!!!!!!!!!!!!!angle: %f saved!\n", calib_valid_data_vec_.back().angle);
  }
}

static nlohmann::json convert_parmas_to_json(const std::array<std::array<Eigen::Vector3d, 2>, 2>& lines_params) {
  std::vector<nlohmann::json> json_pts(4);
  json_pts.at(0) = {lines_params[0][0].x(), lines_params[0][0].y(), lines_params[0][0].z()};
  json_pts.at(1) = {lines_params[0][1].x(), lines_params[0][1].y(), lines_params[0][1].z()};
  json_pts.at(2) = {lines_params[1][0].x(), lines_params[1][0].y(), lines_params[1][0].z()};
  json_pts.at(3) = {lines_params[1][1].x(), lines_params[1][1].y(), lines_params[1][1].z()};
  return json_pts;
}

static nlohmann::json convert_pts_to_json(const std::array<std::array<Eigen::Vector2d, 2>, 2>& pts) {
  std::vector<nlohmann::json> json_pts(4);
  json_pts.at(0) = {pts[0][0].x(), pts[0][0].y()};
  json_pts.at(1) = {pts[0][1].x(), pts[0][1].y()};
  json_pts.at(2) = {pts[1][0].x(), pts[1][0].y()};
  json_pts.at(3) = {pts[1][1].x(), pts[1][1].y()};
  return json_pts;
}

bool TwoLasersCalib::load_calib_data(const std::string& file_path) {
  // 读取文件
  std::ifstream ifs(file_path, std::ios::in);

  nlohmann::json js_whole;
  if (ifs.is_open()) {
    std::cout << "load calib data from " << file_path.c_str() << std::endl;

    // 设置显示格式
    ifs >> js_whole;
    ifs.close();

  } else {
    std::cout << "cannot open file " << file_path.c_str() << std::endl;
    return false;
  }

  if (!js_whole.contains("type") || js_whole["type"] != "two_lasers_calibration") {
    std::cout << "wrong file type!" << std::endl;
    return false;
  }

  // 清空之前的数据
  calib_valid_data_vec_.clear();

  // 加载数据
  for (const auto& data : js_whole["data"].items()) {
    CalibData cd;
    // 中间数据
    std::vector<std::vector<double>> v;
    cd.timestamp = data.value().at("timestamp").get<double>();
    cd.angle = data.value().at("angle").get<double>();

    data.value().at("lines_params").get_to<std::vector<std::vector<double>>>(v);
    cd.lines_params[0][0] = Eigen::Map<Eigen::Vector3d>(v[0].data(), 3);
    cd.lines_params[0][1] = Eigen::Map<Eigen::Vector3d>(v[1].data(), 3);
    cd.lines_params[1][0] = Eigen::Map<Eigen::Vector3d>(v[2].data(), 3);
    cd.lines_params[1][1] = Eigen::Map<Eigen::Vector3d>(v[3].data(), 3);

    v.clear();
    data.value().at("mid_pts_on_line").get_to<std::vector<std::vector<double>>>(v);
    cd.mid_pts_on_line[0][0] = Eigen::Map<Eigen::Vector2d>(v[0].data(), 2);
    cd.mid_pts_on_line[0][1] = Eigen::Map<Eigen::Vector2d>(v[1].data(), 2);
    cd.mid_pts_on_line[1][0] = Eigen::Map<Eigen::Vector2d>(v[2].data(), 2);
    cd.mid_pts_on_line[1][1] = Eigen::Map<Eigen::Vector2d>(v[3].data(), 2);

    calib_valid_data_vec_.push_back(cd);
  }

  return true;
}

bool TwoLasersCalib::save_calib_data(const std::string& file_path) {
  if (calib_valid_data_vec_.empty()) {
    return false;
  }

  nlohmann::json js_whole;
  js_whole["type"] = "two_lasers_calibration";

  // 整理数据
  std::vector<nlohmann::json> js_data;
  for (const auto& data : calib_valid_data_vec_) {
    nlohmann::json js = {{"timestamp", data.timestamp},
                         {"angle", data.angle},
                         {"lines_params", convert_parmas_to_json(data.lines_params)},
                         {"mid_pts_on_line", convert_pts_to_json(data.mid_pts_on_line)}};
    js_data.push_back(js);
  }
  js_whole["data"] = js_data;

  // 保存文件 not std::ios::binary
  std::ofstream ofs(file_path, std::ios::out);

  if (ofs.is_open()) {
    std::cout << "save data to " << file_path.c_str() << std::endl;
    // const auto msgpack = nlohmann::json::to_msgpack(js_data);
    // ofs.write(reinterpret_cast<const char*>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
    // 设置显示格式
    ofs << std::setw(2) << js_whole << std::endl;
    ofs.close();
    return true;
  } else {
    std::cout << "cannot create a file at " << file_path.c_str() << std::endl;
    return false;
  }
}

void TwoLasersCalib::calibration() {
  // 首先获取最新的数据
  update();

  switch (cur_state_) {
    case STATE_IDLE:
      cur_state_ = next_state_;
      break;
    case STATE_START:
      // 都有新数据才计算
      if (laser_insts_[0].is_new_data && laser_insts_[1].is_new_data) {
        laser_insts_[0].is_new_data = false;
        laser_insts_[1].is_new_data = false;
        cur_state_ = STATE_GET_VALID_LINES;
      } else {
        cur_state_ = STATE_IDLE;
      }
      break;
    case STATE_GET_VALID_LINES:
      // 执行任务
      if (task_ptr_->do_task("get_pose_and_points", std::bind(&TwoLasersCalib::get_valid_lines, this))) {  // NOLINT
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          update_show();
          cur_state_ = STATE_CHECK_STEADY;
        } else {
          cur_state_ = STATE_IDLE;
        }
      }
      break;
    case STATE_CHECK_STEADY:
      if (calib_data_vec_.empty()) {
        calib_data_vec_.push_back(calib_data_);
      } else {
        // 检查相机位姿是否一致
        double delta = abs(calib_data_vec_.at(0).angle - calib_data_.angle);
        std::cout << "delta: " << delta << " deg" << std::endl;
        // 抖动小于0.1°
        if (delta < 0.1) {
          calib_data_vec_.push_back(calib_data_);
          // 足够稳定才保存
          if (calib_data_vec_.size() > 4) {
            check_and_save();
          }
        } else {
          // 抖动大则重新开始检测
          calib_data_vec_.clear();
          calib_data_vec_.push_back(calib_data_);
          std::cout << "moved!!!" << std::endl;
        }
      }
      cur_state_ = STATE_IDLE;
      break;
    case STATE_START_CALIB:
      cur_state_ = STATE_IN_CALIB;
      break;
    case STATE_IN_CALIB:
      // 执行任务
      // if (task_ptr_->do_task("calc", std::bind(&CamLaserCalib::calc, this))) {
      //   // 结束后需要读取结果
      //   if (task_ptr_->result<bool>()) {
      //     cur_state_ = STATE_IDLE;
      //   }
      // }
      break;
  }
}

bool TwoLasersCalib::calc() {
  // clang-format off
  int c = 0; int d = 1;
  // clang-format on
}

void TwoLasersCalib::draw_ui() {
  if (!is_show_window_) {
    return;
  }
  // 新建窗口
  ImGui::Begin("Two Lasers Calibration", &is_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  // 激光选择
  draw_sensor_selector<dev::Laser>("laser 1", dev::LASER, laser_insts_[0].laser_dev_ptr);

  // 从文件加载标定数据
  ImGui::SameLine();
  if (ImGui::Button("R")) {
    // 选择加载文件路径
    std::vector<std::string> filters = {"calib data file (.json)", "*.json"};
    std::unique_ptr<pfd::open_file> dialog(new pfd::open_file("choose file", dev::data_default_path, filters));
    while (!dialog->ready()) {
      usleep(1000);
    }

    auto file_paths = dialog->result();
    if (!file_paths.empty()) {
      // 从文件读数据
      if (load_calib_data(file_paths.front())) {
        std::string msg = "load calib data from " + file_paths.front() + " ok!";
        dev::show_pfd_info("load calib data", msg);
      } else {
        std::string msg = "load calib data from " + file_paths.front() + " failed!";
        dev::show_pfd_info("load calib data", msg);
      }
    }
  }
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("load from .json");
  }

  if (!calib_valid_data_vec_.empty()) {
    ImGui::SameLine();
    // 保存标定数据
    if (ImGui::Button("W")) {
      // 选择保存文件路径
      std::vector<std::string> filters = {"calib data file (.json)", "*.json"};
      std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file", dev::data_default_path, filters));
      while (!dialog->ready()) {
        usleep(1000);
      }

      auto file_path = dialog->result();
      if (!file_path.empty()) {
        // 导出标定数据
        if (!save_calib_data(file_path)) {
          std::string msg = "save calib data to " + file_path + " failed!";
          dev::show_pfd_info("save calib data", msg);
        } else {
          std::string msg = "save calib data to " + file_path + " ok!";
          dev::show_pfd_info("save calib data", msg);
        }
      }
    }

    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("save calib data to .json file");
    }
  }

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
          // 清空上次的标定数据
          calib_valid_data_vec_.clear();
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

    // 大于6帧数据就可以开始进行标定操作了
    if (calib_valid_data_vec_.size() > 6) {
      ImGui::SameLine();
      // 开始执行标定
      if (ImGui::Button("calib")) {
        next_state_ = STATE_START_CALIB;
      }
    }
  }

  // 显示当前有效数据个数
  ImGui::Separator();
  ImGui::TextDisabled("vaild: %zu", calib_valid_data_vec_.size());
  ImGui::End();

  // 显示图像
  if (is_show_image_) {
    laser_insts_[0].img_show_dev_ptr->show_image(is_show_image_);
    laser_insts_[1].img_show_dev_ptr->show_image(is_show_image_);
  }
}

}  // namespace calibration
