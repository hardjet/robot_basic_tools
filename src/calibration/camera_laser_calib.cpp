#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include "imgui.h"
#include "portable-file-dialogs.h"
#include "nlohmann/json.hpp"

#include "dev/april_board.hpp"
#include "dev/camera.hpp"
#include "dev/laser.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "dev/util.hpp"
#include "calibration/task_back_ground.hpp"
#include "calibration/camera_laser_calib.hpp"
#include "algorithm/line.h"
#include "algorithm/util.h"
#include "algorithm/laser_cam_ceres.h"
#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"
#include "camera_model/camera_models/Camera.h"

// ---- 相机-单线激光标定状态
// 空闲
#define STATE_IDLE 0
// 启动
#define STATE_START 1
// 获取相机位姿和落在标定板上的激光点
#define STATE_GET_POSE_AND_PTS 2
// 检查数据稳定性,连续5帧姿态没有大的变化，取中间帧作为候选
#define STATE_CHECK_STEADY 3
// 开始标定
#define STATE_START_CALIB 4
// 在标定过程中
#define STATE_IN_CALIB 5

namespace calibration {

CamLaserCalib::CamLaserCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                             std::shared_ptr<dev::AprilBoard>& april_board_ptr_)
    : BaseCalib(sensor_manager_ptr), april_board_ptr_(april_board_ptr_) {
  // 图像显示
  im_show_dev_ptr_ = std::make_shared<dev::ImageShow>();
  // 激光以图像显示
  laser_show_dev_ptr_ = std::make_shared<dev::ImageShow>();
  // 后台任务
  task_ptr_ = std::make_shared<calibration::Task>();
  // 当前标定数据
  calib_data_ = std::make_shared<CalibData>();
}

void CamLaserCalib::draw_gl(glk::GLSLShader& shader) {}

void CamLaserCalib::update_data() {
  std::lock_guard<std::mutex> lock(mtx_);
  // 图像
  if (!image_ptr_) {
    image_ptr_ = cam_ptr_->data();
    if (image_ptr_) {
      is_new_image_ = true;
    }
  } else {
    auto image = cam_ptr_->data();
    if (image_ptr_->header.stamp.nsec != image->header.stamp.nsec) {
      image_ptr_ = image;
      is_new_image_ = true;
    }
  }

  // 激光
  if (!laser_data_ptr_) {
    laser_data_ptr_ = laser_ptr_->data();
    if (laser_data_ptr_) {
      is_new_laser_ = true;
    }
  } else {
    auto laser = laser_ptr_->data();
    if (laser_data_ptr_->header.stamp.nsec != laser->header.stamp.nsec) {
      laser_data_ptr_ = laser;
      is_new_laser_ = true;
    }
  }
}

bool CamLaserCalib::get_pose_and_points() {
  // 检测到的角点空间坐标
  std::vector<cv::Point2f> imagePoints;
  // 检测到的角点图像坐标
  std::vector<cv::Point3f> objectPoints;
  // 相机坐标系到世界坐标系的变换
  Eigen::Matrix4d Twc;

  // 当前处理的图像以及激光数据
  boost::shared_ptr<const cv_bridge::CvImage> cur_image{nullptr};
  boost::shared_ptr<const sensor_msgs::LaserScan> cur_laser_data{nullptr};

  auto start_time = ros::WallTime::now();
  // 将图像和激光锁定
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cur_image.reset(new const cv_bridge::CvImage(image_ptr_->header, image_ptr_->encoding, image_ptr_->image));
    cur_laser_data = laser_data_ptr_;
  }

  cv::Mat img;
  // 需要转为灰度
  if (cur_image->image.channels() == 3) {
    cv::cvtColor(cur_image->image, img, cv::COLOR_RGB2GRAY);
  } else {
    img = cur_image->image.clone();
  }

  cv::Mat img_show;
  // 需要转为彩色图像
  if (cur_image->image.channels() == 1) {
    cv::cvtColor(cur_image->image, img_show, cv::COLOR_GRAY2RGB);
  } else {
    img_show = cur_image->image.clone();
  }

  if (april_board_ptr_->board->computeObservation(img, img_show, objectPoints, imagePoints)) {
    // 计算外参T_wc cam->world
    cam_ptr_->cam()->estimateExtrinsics(objectPoints, imagePoints, Twc, img_show);
    calib_data_->timestamp = cur_image->header.stamp.toSec();
    calib_data_->t_wc = Twc.block(0, 3, 3, 1);
    Eigen::Matrix3d R_wc = Twc.block(0, 0, 3, 3);
    Eigen::Quaterniond q_wc(R_wc);
    calib_data_->q_wc = q_wc;

    show_cam_img_ptr_.reset(new const cv_bridge::CvImage(cur_image->header, cur_image->encoding, img_show));
    calib_data_->cam_img_ptr_ = show_cam_img_ptr_;

    // 检测激光中的直线
    cv::Mat laser_show;
    algorithm::Line line(*cur_laser_data, 60.0, 2.0);
    if (line.find_line(calib_data_->line_pts, laser_show)) {
      auto end_time = ros::WallTime::now();
      double time_used = (end_time - start_time).toSec() * 1000;

      // 显示用时
      cv::putText(laser_show, "time used (ms):  " + std::to_string(time_used), cv::Point2f(10, 20),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));

      show_laser_img_ptr_.reset(new const cv_bridge::CvImage(cur_laser_data->header, cur_image->encoding, laser_show));
      calib_data_->laser_img_ptr_ = show_laser_img_ptr_;
      return true;
    } else {
      return false;
    }

  } else {
    // std::cout << "no april board detected!" << std::endl;
    return false;
  }
}

void CamLaserCalib::check_and_save() {
  bool is_need_to_save = true;
  // 逐个检测角度值
  for (auto& data : calib_valid_data_vec_) {
    double theta = 2 * std::acos((data.q_wc.inverse() * calib_data_vec_.at(2).q_wc).w());
    // 保证间隔5°以上
    if (theta < DEG2RAD_RBT(5.0)) {
      is_need_to_save = false;
      break;
    }
  }

  if (is_need_to_save) {
    // 取第3帧保存
    calib_valid_data_vec_.push_back(calib_data_vec_.at(2));
  }
}

static void convert_json_to_pts(std::vector<Eigen::Vector3d>& pts, nlohmann::json& js) {
  pts.resize(js.size());
  for (unsigned int idx = 0; idx < js.size(); ++idx) {
    std::vector<double> v;
    js.at(idx).get_to<std::vector<double>>(v);
    pts.at(idx) = Eigen::Map<Eigen::VectorXd>(v.data(), 3);
  }
}

bool CamLaserCalib::load_calib_data(const std::string& file_path) {
  // 清空之前的数据
  calib_valid_data_vec_.clear();

  // 读取文件
  std::ifstream ifs(file_path, std::ios::in);

  nlohmann::json js_data;
  if (ifs.is_open()) {
    std::cout << "load calib data from " << file_path.c_str() << std::endl;

    // 设置显示格式
    ifs >> js_data;
    ifs.close();

  } else {
    std::cout << "cannot open file " << file_path.c_str() << std::endl;
    return false;
  }

  // 加载数据
  for (const auto& data : js_data.items()) {
    CalibData d;
    std::vector<double> v;
    d.timestamp = data.value().at("timestamp").get<double>();
    data.value().at("q_wc").get_to<std::vector<double>>(v);
    d.q_wc.x() = v[0];
    d.q_wc.y() = v[1];
    d.q_wc.z() = v[2];
    d.q_wc.w() = v[3];
    v.clear();
    data.value().at("t_wc").get_to<std::vector<double>>(v);
    d.t_wc = Eigen::Map<Eigen::VectorXd>(v.data(), 3);
    // 加载直线点
    convert_json_to_pts(d.line_pts, data.value().at("line_pts"));
    calib_valid_data_vec_.push_back(d);
  }

  return true;
}

static nlohmann::json convert_pts_to_json(const std::vector<Eigen::Vector3d>& pts) {
  std::vector<nlohmann::json> json_pts(pts.size());
  for (unsigned int idx = 0; idx < pts.size(); ++idx) {
    json_pts.at(idx) = {pts.at(idx).x(), pts.at(idx).y(), pts.at(idx).z()};
  }
  return json_pts;
}

bool CamLaserCalib::save_calib_data(const std::string& file_path) {
  if (calib_valid_data_vec_.empty()) {
    return false;
  }

  // 整理数据
  std::vector<nlohmann::json> js_data;
  for (const auto& data : calib_valid_data_vec_) {
    nlohmann::json js = {{"timestamp", data.timestamp},
                         {"q_wc", {data.q_wc.x(), data.q_wc.y(), data.q_wc.z(), data.q_wc.w()}},
                         {"t_wc", {data.t_wc.x(), data.t_wc.y(), data.t_wc.z()}},
                         {"line_pts", convert_pts_to_json(data.line_pts)}};
    js_data.push_back(js);
  }

  // 保存文件 not std::ios::binary
  std::ofstream ofs(file_path, std::ios::out);

  if (ofs.is_open()) {
    std::cout << "save data to " << file_path.c_str() << std::endl;
    // const auto msgpack = nlohmann::json::to_msgpack(js_data);
    // ofs.write(reinterpret_cast<const char*>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
    // 设置显示格式
    ofs << std::setw(2) << js_data << std::endl;
    ofs.close();
    return true;
  } else {
    std::cout << "cannot create a file at " << file_path.c_str() << std::endl;
    return false;
  }
}

bool CamLaserCalib::calc() {
  // 准备标定数据
  std::vector<algorithm::Oberserve> obs;

  for (const auto& data : calib_valid_data_vec_) {
    Eigen::Vector2d line;
    algorithm::LineFittingCeres(data.line_pts, line);
    std::vector<Eigen::Vector3d> points_on_line;

    // 激光所在直线不能垂直于某个轴
    double x_start(data.line_pts.begin()->x()), x_end(data.line_pts.end()->x());
    double y_start(data.line_pts.begin()->y()), y_end(data.line_pts.end()->y());
    if (fabs(x_end - x_start) > fabs(y_end - y_start)) {
      y_start = -(x_start * line(0) + 1) / line(1);
      y_end = -(x_end * line(0) + 1) / line(1);
      // 可能垂直于 x 轴，采用y值来计算 x
    } else {
      x_start = -(y_start * line(1) + 1) / line(0);
      x_end = -(y_end * line(1) + 1) / line(0);
    }

    points_on_line.emplace_back(x_start, y_start, 0);
    points_on_line.emplace_back(x_end, y_end, 0);

    algorithm::Oberserve ob;
    ob.tag_pose_q_ca = data.q_wc.inverse();
    ob.tag_pose_t_ca = -ob.tag_pose_q_ca.toRotationMatrix() * data.t_wc;
    ob.points = data.line_pts;
    ob.points_on_line = points_on_line;
    obs.push_back(ob);
  }

  Eigen::Matrix4d Tlc_initial = Eigen::Matrix4d::Identity();
  algorithm::CamLaserCalClosedSolution(obs, Tlc_initial);

  Eigen::Matrix4d Tcl = Tlc_initial.inverse();
  algorithm::CamLaserCalibration(obs, Tcl, true);

  std::cout << "\n----- Transform from Camera to Laser Tlc is: -----\n" << std::endl;
  Eigen::Matrix4d Tlc = Tcl.inverse();
  std::cout << Tlc << std::endl;

  std::cout << "\n----- Transform from Camera to Laser, euler angles and translations are: -----\n" << std::endl;
  Eigen::Matrix3d Rlc(Tlc.block(0, 0, 3, 3));
  Eigen::Vector3d tlc(Tlc.block(0, 3, 3, 1));
  Eigen::Quaterniond q(Rlc);
  algorithm::EulerAngles rpy = algorithm::ToEulerAngles(q);
  std::cout << "q(x, y, z, w):" << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  std::cout << "q(x, y, z, w):" << q.coeffs().transpose() << std::endl;
  std::cout << "   roll(rad): " << rpy.roll << " pitch(rad): " << rpy.pitch << " yaw(rad): " << rpy.yaw << "\n"
            << "or roll(deg): " << rpy.roll * 180. / M_PI << " pitch(deg): " << rpy.pitch * 180. / M_PI
            << " yaw(deg): " << rpy.yaw * 180. / M_PI << "\n"
            << "       tx(m): " << tlc.x() << "  ty(m): " << tlc.y() << "   tz(m): " << tlc.z() << std::endl;

  return true;
}

void CamLaserCalib::calibration() {
  // 首先获取最新的数据
  update_data();

  switch (cur_state_) {
    case STATE_IDLE:
      cur_state_ = next_state_;
      break;
    case STATE_START:
      if (is_new_image_ && is_new_laser_) {
        is_new_image_ = false;
        is_new_laser_ = false;
        cur_state_ = STATE_GET_POSE_AND_PTS;
      } else {
        cur_state_ = STATE_IDLE;
      }
      break;
    case STATE_GET_POSE_AND_PTS:
      // 执行任务
      if (task_ptr_->do_task("get_pose_and_points", std::bind(&CamLaserCalib::get_pose_and_points, this))) {
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          im_show_dev_ptr_->update_image(show_cam_img_ptr_);
          laser_show_dev_ptr_->update_image(show_laser_img_ptr_);
          cur_state_ = STATE_CHECK_STEADY;
        } else {
          cur_state_ = STATE_IDLE;
        }
      }
      break;
    case STATE_CHECK_STEADY:
      if (calib_data_vec_.empty()) {
        calib_data_vec_.push_back(*calib_data_);
      } else {
        // 检查相机位姿是否一致
        double dist = (calib_data_vec_.at(0).t_wc - calib_data_->t_wc).norm();
        // 四元数的转角是原本的1/2
        double theta = 2 * std::acos((calib_data_vec_.at(0).q_wc.inverse() * calib_data_->q_wc).w());
        std::cout << "dist:" << dist << ", theta:" << RAD2DEG_RBT(theta) << std::endl;
        // 抖动小于1cm与0.5°
        if (dist < 0.01 && theta < DEG2RAD_RBT(0.8)) {
          calib_data_vec_.push_back(*calib_data_);
          // 足够稳定才保存
          if (calib_data_vec_.size() > 3) {
            check_and_save();
          }
        } else {
          // 抖动大则重新开始检测
          calib_data_vec_.clear();
          calib_data_vec_.push_back(*calib_data_);
          std::cout << "moved!!!" << std::endl;
        }
        cur_state_ = STATE_IDLE;
      }
      break;
    case STATE_START_CALIB:
      cur_state_ = STATE_IN_CALIB;
      break;
    case STATE_IN_CALIB:
      // 执行任务
      if (task_ptr_->do_task("calc", std::bind(&CamLaserCalib::calc, this))) {
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          cur_state_ = STATE_IDLE;
        }
      }
      break;
  }
}

void CamLaserCalib::draw_ui() {
  if (!is_show_window_) {
    return;
  }
  // 新建窗口
  ImGui::Begin("Camera and Laser Calibration", &is_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  // 相机选择
  draw_sensor_selector<dev::Camera>("camera", dev::CAMERA, cam_ptr_);

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
        }
      }
    }

    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("save calib data to .json file");
    }
  }

  // 激光选择
  draw_sensor_selector<dev::Laser>("laser", dev::LASER, laser_ptr_);

  // 设备就绪后才能标定
  if (cam_ptr_ && laser_ptr_) {
    ImGui::SameLine();
    // 选择是否显示图像
    if (ImGui::Checkbox("show image", &is_show_image_)) {
      if (is_show_image_) {
        im_show_dev_ptr_->enable("calib cam", false);
        laser_show_dev_ptr_->enable("calib laser", false);
      } else {
        im_show_dev_ptr_->disable();
        laser_show_dev_ptr_->disable();
      }
    }

    // 标定逻辑
    calibration();

    if (next_state_ == STATE_IDLE) {
      if (ImGui::Button("start")) {
        next_state_ = STATE_START;
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
  ImGui::TextDisabled("vaild: %zu", calib_valid_data_vec_.size());

  ImGui::End();

  // 显示图像
  if (is_show_image_) {
    im_show_dev_ptr_->show_image(is_show_image_);
    laser_show_dev_ptr_->show_image(is_show_image_);
  }
}

}  // namespace calibration
