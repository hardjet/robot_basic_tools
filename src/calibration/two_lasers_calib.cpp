#include <fstream>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "imgui.h"

#include "portable-file-dialogs.h"
#include "nlohmann/json.hpp"

#include "glk/simple_lines.hpp"
#include "glk/primitives/primitives.hpp"
#include "glk/glsl_shader.hpp"

#include "dev/laser.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "dev/util.hpp"

#include "calibration/task_back_ground.hpp"
#include "calibration/two_lasers_calib.hpp"

#include "algorithm/line_detect.h"
#include "algorithm/two_lasers_ceres.h"
#include "algorithm/util.h"

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

  // 设置相对位姿初始值
  T12_ = Eigen::Matrix4f::Identity();

  // // 设置旋转矩阵初始值
  // // Eigen::Quaternionf q_init{0.805, 0.000, 0.593, 0.000};
  //
  // Eigen::Quaternionf q_init = algorithm::ypr2quaternion(0,DEG2RAD_RBT(60),0).cast<float>();
  // T12_.block<3, 3>(0, 0) = q_init.toRotationMatrix();
  // // 设置平移向量
  // T12_.block<3, 1>(0, 3) = Eigen::Vector3f{0.0, 0.0, 1.248};
  //
  // std::cout << "T12_:\n" << T12_ << std::endl;

  // 测试
  // Eigen::Vector3d l1{1, 2, 3};
  // Eigen::Vector3d l2{-1, 6, 4};
  // Eigen::Vector3d l3{7, 9, -20};
  //
  // Eigen::Vector3d a = algorithm::skew_symmetric(l1) * (algorithm::skew_symmetric(l2) * l3);
  // Eigen::Vector3d b = algorithm::skew_symmetric(l1) * algorithm::skew_symmetric(l2) * l3;
  // std::cout << "a: " << a.transpose() << "b: " << b.transpose() << std::endl;

  // std::cout << "l1 x l2 " << l1.cross(l2).transpose() << "l1^l2: " << (algorithm::skew_symmetric(l1) *
  // l2).transpose() << std::endl;
}

void TwoLasersCalib::draw_calib_data(glk::GLSLShader& shader) {
  if (!b_show_calib_data_) {
    return;
  }

  // 更新显示直线
  if (b_need_to_update_cd_) {
    b_need_to_update_cd_ = false;
    // std::cout << "update calib data" << std::endl;
    // 3d空间直线信息
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices(4);
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors(4);
    std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>> infos(4);

    const auto& cur_calib_data = calib_valid_data_vec_[selected_calib_data_id_ - 1];
    vertices.at(0) = cur_calib_data.lines_pts[0][0][0];
    vertices.at(1) = cur_calib_data.lines_pts[0][0][1];
    vertices.at(2) = cur_calib_data.lines_pts[0][1][0];
    vertices.at(3) = cur_calib_data.lines_pts[0][1][1];

    colors.at(0) = Eigen::Vector4f(0.f, 255.f, 255.f, 1.0f);
    colors.at(1) = Eigen::Vector4f(0.f, 255.f, 255.f, 1.0f);
    colors.at(2) = Eigen::Vector4f(255.f, 0.f, 255.f, 1.0f);
    colors.at(3) = Eigen::Vector4f(255.f, 0.f, 255.f, 1.0f);

    infos.at(0) = Eigen::Vector4i(0, 0, 1, 0);
    infos.at(1) = Eigen::Vector4i(0, 0, 1, 0);
    infos.at(2) = Eigen::Vector4i(0, 0, 1, 0);
    infos.at(3) = Eigen::Vector4i(0, 0, 1, 0);

    calib_show_data_[0].laser_lines_drawable_ptr.reset(new glk::SimpleLines(vertices, colors, infos));
    calib_show_data_[0].mid_pt_on_lines[0] = cur_calib_data.mid_pt_on_lines[0][0];
    calib_show_data_[0].mid_pt_on_lines[1] = cur_calib_data.mid_pt_on_lines[0][1];

    vertices.at(0) = cur_calib_data.lines_pts[1][0][0];
    vertices.at(1) = cur_calib_data.lines_pts[1][0][1];
    vertices.at(2) = cur_calib_data.lines_pts[1][1][0];
    vertices.at(3) = cur_calib_data.lines_pts[1][1][1];

    infos.at(0) = Eigen::Vector4i(0, 0, 2, 0);
    infos.at(1) = Eigen::Vector4i(0, 0, 2, 0);
    infos.at(2) = Eigen::Vector4i(0, 0, 2, 0);
    infos.at(3) = Eigen::Vector4i(0, 0, 2, 0);

    calib_show_data_[1].laser_lines_drawable_ptr.reset(new glk::SimpleLines(vertices, colors, infos));
    calib_show_data_[1].mid_pt_on_lines[0] = cur_calib_data.mid_pt_on_lines[1][0];
    calib_show_data_[1].mid_pt_on_lines[1] = cur_calib_data.mid_pt_on_lines[1][1];
  }

  // 绘图
  // 画直线上的中点
  auto draw_pt = [&shader](Eigen::Matrix4f model, const Eigen::Vector3d& pos, const Eigen::Vector4f& color) {
    model.block<3, 1>(0, 3) = model.block<3, 3>(0, 0) * pos.cast<float>() + model.block<3, 1>(0, 3);

    // 画直线中点
    // 更改大小 设置球的半径大小
    model.block<3, 3>(0, 0) *= 0.025;
    // 改变位置
    // model.block<3, 1>(0, 3) = Eigen::Vector3f{float(pos.x()), float(pos.y()), float(pos.z())};
    shader.set_uniform("color_mode", 1);
    shader.set_uniform("model_matrix", model);
    // 设置显示颜色
    shader.set_uniform("material_color", color);

    const auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    sphere.draw(shader);
  };

  // 后面需要设置为激光0的位姿
  Eigen::Matrix4f model_matrix = Eigen::Matrix4f::Identity();
  shader.set_uniform("model_matrix", model_matrix);
  shader.set_uniform("color_mode", 2);
  calib_show_data_[0].laser_lines_drawable_ptr->draw(shader);

  // model_matrix = Eigen::Matrix4f::Identity();
  draw_pt(model_matrix, calib_show_data_[0].mid_pt_on_lines[0], Eigen::Vector4f(0.f, 255.f, 255.f, 1.0f));
  // model_matrix = Eigen::Matrix4f::Identity();
  draw_pt(model_matrix, calib_show_data_[0].mid_pt_on_lines[1], Eigen::Vector4f(255.f, 0.f, 255.f, 1.0f));

  // 后面需要设置为激光1的位姿
  model_matrix = T12_;
  shader.set_uniform("model_matrix", model_matrix);
  shader.set_uniform("color_mode", 2);
  calib_show_data_[1].laser_lines_drawable_ptr->draw(shader);

  // model_matrix = T12_;
  draw_pt(model_matrix, calib_show_data_[1].mid_pt_on_lines[0], Eigen::Vector4f(0.f, 255.f, 255.f, 1.0f));
  // model_matrix = T12_;
  draw_pt(model_matrix, calib_show_data_[1].mid_pt_on_lines[1], Eigen::Vector4f(255.f, 0.f, 255.f, 1.0f));
}

void TwoLasersCalib::draw_gl(glk::GLSLShader& shader) {
  draw_calib_data(shader);

  if (next_state_ != STATE_START || !laser_insts_[0].laser_lines_drawable_ptr) {
    return;
  }

  shader.set_uniform("color_mode", 2);

  for (const auto& laser_inst : laser_insts_) {
    shader.set_uniform("model_matrix", laser_inst.laser_dev_ptr->get_sensor_pose());
    laser_inst.laser_lines_drawable_ptr->draw(shader);
  }
}

void TwoLasersCalib::update_show() {
  // 3d空间直线信息
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices(4);
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;
  std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>> infos;

  double x_start, x_end;
  double y_start, y_end;

  // 设置标定数据
  // 时间
  calib_data_.timestamp = laser_insts_[0].calib_data_ptr->header.stamp.toSec();

  bool b_need_to_swap{false};
  // 需要判断是否需要调整保存顺序, 同一边的激光对应好
  // 激光0序号0直线的角度
  double theta_0_0 = atan(-laser_insts_[0].lines_params[0](0) / laser_insts_[0].lines_params[0](1));
  // 激光1序号0直线的角度
  double theta_1_0 = atan(-laser_insts_[1].lines_params[0](0) / laser_insts_[1].lines_params[0](1));
  // 激光1序号1直线的角度
  double theta_1_1 = atan(-laser_insts_[1].lines_params[1](0) / laser_insts_[1].lines_params[1](1));
  // 同一边的直线角度差一定比不同边直线角度差小
  // 需要变换
  if (abs(theta_0_0 - theta_1_0) > abs(theta_0_0 - theta_1_1)) {
    b_need_to_swap = true;
  }
  // printf("need_to_swap: %d, 0-0: %.3f, 1-0: %.3f, 1-1: %.3f\n", b_need_to_swap, RAD2DEG_RBT(theta_0_0),
  //        RAD2DEG_RBT(theta_1_0), RAD2DEG_RBT(theta_1_1));
  if (b_need_to_swap) {
    std::cout << "need_to_swap!!!!" << std::endl;
  }

  for (auto& laser_inst : laser_insts_) {
    // 更新显示图象
    laser_inst.img_show_dev_ptr->update_image(laser_inst.img_ptr);

    // 计算3d空间直线顶点坐标
    colors.clear();
    infos.clear();

    // 外扩的长度
    const double extend_len = 0.4;
    // 直线与x轴的夹角
    double theta, delta_len;
    // 设置直线参数
    if (laser_inst.id == 1 && b_need_to_swap) {
      calib_data_.lines_params[1][0] = laser_inst.lines_params[1];
      calib_data_.lines_params[1][1] = laser_inst.lines_params[0];
    } else {
      calib_data_.lines_params[laser_inst.id] = laser_inst.lines_params;
    }

    for (int i = 0; i < 2; i++) {
      const auto& line_params = laser_inst.lines_params[i];
      const auto& line_min_max = laser_inst.lines_min_max[i];

      // 计算角度
      theta = atan(-line_params(0) / line_params(1));
      delta_len = abs(extend_len * sin(theta));
      // printf("theta: %f, delta_len: %f\n", theta * 180. / M_PI, delta_len);

      // 外扩一定长度
      y_start = line_min_max(0) - delta_len;
      y_end = line_min_max(1) + delta_len;

      x_start = -(line_params(1) * y_start + line_params(2)) / line_params(0);
      x_end = -(line_params(1) * y_end + line_params(2)) / line_params(0);

      // 计算直线上的中点
      double mid_y = (line_min_max(0) + line_min_max(1)) / 2.;
      double mid_x = -(line_params(1) * mid_y + line_params(2)) / line_params(0);

      // 添加直线顶点
      if (laser_inst.id == 1 && b_need_to_swap) {
        vertices[(1 - i) * 2 + 0] = Eigen::Vector3f{float(x_start), float(y_start), 0};
        vertices[(1 - i) * 2 + 1] = Eigen::Vector3f{float(x_end), float(y_end), 0};
        calib_data_.mid_pt_on_lines[laser_inst.id][1 - i] = Eigen::Vector3d{mid_x, mid_y, 0};
        calib_data_.lines_pts[laser_inst.id][1 - i][0] = Eigen::Vector3f{float(x_start), float(y_start), 0};
        calib_data_.lines_pts[laser_inst.id][1 - i][1] = Eigen::Vector3f{float(x_end), float(y_end), 0};
      } else {
        vertices[i * 2 + 0] = Eigen::Vector3f{float(x_start), float(y_start), 0};
        vertices[i * 2 + 1] = Eigen::Vector3f{float(x_end), float(y_end), 0};
        calib_data_.mid_pt_on_lines[laser_inst.id][i] = Eigen::Vector3d{mid_x, mid_y, 0};
        calib_data_.lines_pts[laser_inst.id][i][0] = Eigen::Vector3f{float(x_start), float(y_start), 0};
        calib_data_.lines_pts[laser_inst.id][i][1] = Eigen::Vector3f{float(x_end), float(y_end), 0};
      }
    }

    // 添加额外信息
    infos.emplace_back(laser_inst.laser_dev_ptr->sensor_id, laser_inst.id, 1, 0);
    infos.emplace_back(laser_inst.laser_dev_ptr->sensor_id, laser_inst.id, 2, 0);
    infos.emplace_back(laser_inst.laser_dev_ptr->sensor_id, laser_inst.id, 1, 0);
    infos.emplace_back(laser_inst.laser_dev_ptr->sensor_id, laser_inst.id, 2, 0);

    // 添加颜色信息
    colors.emplace_back(0.f, 255.f, 255.f, 1.0f);
    colors.emplace_back(0.f, 255.f, 255.f, 1.0f);
    colors.emplace_back(255.f, 0.f, 255.f, 1.0f);
    colors.emplace_back(255.f, 0.f, 255.f, 1.0f);

    // 重置3d直线
    laser_inst.laser_lines_drawable_ptr.reset(new glk::SimpleLines(vertices, colors, infos));
    // std::cout << "laser_lines_drawable_ptr: " << laser_inst.laser_lines_drawable_ptr << std::endl;
  }

  // 设置角度
  calib_data_.angle = RAD2DEG_RBT(theta_0_0);
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
    algorithm::LineDetect line_detector(*laser_inst.calib_data_ptr, angle_range_, max_range_);
    if (line_detector.find_two_lines(laser_inst.lines_params, laser_inst.lines_min_max, laser_img_show)) {
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
  bool b_need_to_save = true;
  // 逐个检测角度值与第3帧比较
  for (auto& data : calib_valid_data_vec_) {
    double theta = abs(data.angle - calib_data_vec_.at(3).angle);
    // 保证间隔x°以上
    if (theta < between_angle_) {
      b_need_to_save = false;
      break;
    }
  }

  if (b_need_to_save) {
    // 取第3帧保存
    calib_valid_data_vec_.push_back(calib_data_vec_.at(3));
    printf("!!!!!!!!!!!!!angle: %f saved!\n", calib_valid_data_vec_.back().angle);
  }
}

static nlohmann::json convert_parmas_to_json(const std::array<std::array<Eigen::Vector3d, 2>, 2>& lines_params) {
  std::vector<nlohmann::json> json_pts(4);
  // 激光0 直线0
  json_pts.at(0) = {lines_params[0][0].x(), lines_params[0][0].y(), lines_params[0][0].z()};
  // 激光0 直线1
  json_pts.at(1) = {lines_params[0][1].x(), lines_params[0][1].y(), lines_params[0][1].z()};
  // 激光1 直线0
  json_pts.at(2) = {lines_params[1][0].x(), lines_params[1][0].y(), lines_params[1][0].z()};
  // 激光1 直线1
  json_pts.at(3) = {lines_params[1][1].x(), lines_params[1][1].y(), lines_params[1][1].z()};
  return json_pts;
}

static nlohmann::json convert_pts_to_json(const std::array<std::array<Eigen::Vector3d, 2>, 2>& pts) {
  std::vector<nlohmann::json> json_pts(4);
  // 激光0 直线0
  json_pts.at(0) = {pts[0][0].x(), pts[0][0].y(), pts[0][0].z()};
  // 激光0 直线1
  json_pts.at(1) = {pts[0][1].x(), pts[0][1].y(), pts[0][1].z()};
  // 激光1 直线0
  json_pts.at(2) = {pts[1][0].x(), pts[1][0].y(), pts[1][0].z()};
  // 激光1 直线1
  json_pts.at(3) = {pts[1][1].x(), pts[1][1].y(), pts[1][1].z()};
  return json_pts;
}

static nlohmann::json convert_pts_to_json(const std::array<std::array<std::array<Eigen::Vector3f, 2>, 2>, 2>& pts) {
  std::vector<nlohmann::json> json_pts(8);
  // 激光0 直线0
  json_pts.at(0) = {pts[0][0][0].x(), pts[0][0][0].y(), pts[0][0][0].z()};
  json_pts.at(1) = {pts[0][0][1].x(), pts[0][0][1].y(), pts[0][0][1].z()};
  // 激光0 直线1
  json_pts.at(2) = {pts[0][1][0].x(), pts[0][1][0].y(), pts[0][1][0].z()};
  json_pts.at(3) = {pts[0][1][1].x(), pts[0][1][1].y(), pts[0][1][1].z()};
  // 激光1 直线0
  json_pts.at(4) = {pts[1][0][0].x(), pts[1][0][0].y(), pts[1][0][0].z()};
  json_pts.at(5) = {pts[1][0][1].x(), pts[1][0][1].y(), pts[1][0][1].z()};
  // 激光1 直线1
  json_pts.at(6) = {pts[1][1][0].x(), pts[1][1][0].y(), pts[1][1][0].z()};
  json_pts.at(7) = {pts[1][1][1].x(), pts[1][1][1].y(), pts[1][1][1].z()};
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
    data.value().at("mid_pt_on_lines").get_to<std::vector<std::vector<double>>>(v);
    cd.mid_pt_on_lines[0][0] = Eigen::Map<Eigen::Vector3d>(v[0].data(), 3);
    cd.mid_pt_on_lines[0][1] = Eigen::Map<Eigen::Vector3d>(v[1].data(), 3);
    cd.mid_pt_on_lines[1][0] = Eigen::Map<Eigen::Vector3d>(v[2].data(), 3);
    cd.mid_pt_on_lines[1][1] = Eigen::Map<Eigen::Vector3d>(v[3].data(), 3);

    std::vector<std::vector<float>> v_f;
    data.value().at("lines_pts").get_to<std::vector<std::vector<float>>>(v_f);
    cd.lines_pts[0][0][0] = Eigen::Map<Eigen::Vector3f>(v_f[0].data(), 3);
    cd.lines_pts[0][0][1] = Eigen::Map<Eigen::Vector3f>(v_f[1].data(), 3);

    cd.lines_pts[0][1][0] = Eigen::Map<Eigen::Vector3f>(v_f[2].data(), 3);
    cd.lines_pts[0][1][1] = Eigen::Map<Eigen::Vector3f>(v_f[3].data(), 3);

    cd.lines_pts[1][0][0] = Eigen::Map<Eigen::Vector3f>(v_f[4].data(), 3);
    cd.lines_pts[1][0][1] = Eigen::Map<Eigen::Vector3f>(v_f[5].data(), 3);

    cd.lines_pts[1][1][0] = Eigen::Map<Eigen::Vector3f>(v_f[6].data(), 3);
    cd.lines_pts[1][1][1] = Eigen::Map<Eigen::Vector3f>(v_f[7].data(), 3);

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
                         {"mid_pt_on_lines", convert_pts_to_json(data.mid_pt_on_lines)},
                         {"lines_pts", convert_pts_to_json(data.lines_pts)}};
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
          if (calib_data_vec_.size() > 6) {
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
      if (task_ptr_->do_task("calc", std::bind(&TwoLasersCalib::calc, this))) {  // NOLINT
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          // 将计算结果更新到laser2
          update_laser2_pose();
          update_ui_transform();
          cur_state_ = STATE_IDLE;
        }
      }
      break;
  }
}

bool TwoLasersCalib::calc() {
  // 准备标定数据
  std::vector<algorithm::Observation> obs;

  double scale = 1. / sqrt(calib_valid_data_vec_.size());
  std::cout << "scale: " << scale << std::endl;

  // 直线的方向向量 {B, -A, 0}
  Eigen::Vector3d l;
  for (const auto& data : calib_valid_data_vec_) {
    algorithm::Observation ob;
    // 直线归一化
    // ob.l_1_a = data.lines_params[0][0].normalized();
    // ob.c_1_a = data.mid_pt_on_lines[0][0];
    // ob.l_1_b = data.lines_params[0][1].normalized();
    // ob.c_1_b = data.mid_pt_on_lines[0][1];
    //
    // ob.l_2_a = data.lines_params[1][0].normalized();
    // ob.c_2_a = data.mid_pt_on_lines[1][0];
    // ob.l_2_b = data.lines_params[1][1].normalized();
    // ob.c_2_b = data.mid_pt_on_lines[1][1];

    l = Eigen::Vector3d{data.lines_params[0][0](1), -data.lines_params[0][0](0), 0.};
    ob.l_1_a = l;
    ob.c_1_a = data.mid_pt_on_lines[0][0];
    l = Eigen::Vector3d{data.lines_params[0][1](1), -data.lines_params[0][1](0), 0.};
    ob.l_1_b = l;
    ob.c_1_b = data.mid_pt_on_lines[0][1];

    l = Eigen::Vector3d{data.lines_params[1][0](1), -data.lines_params[1][0](0), 0.};
    ob.l_2_a = l;
    ob.c_2_a = data.mid_pt_on_lines[1][0];
    l = Eigen::Vector3d{data.lines_params[1][1](1), -data.lines_params[1][1](0), 0.};
    ob.l_2_b = l;
    ob.c_2_b = data.mid_pt_on_lines[1][1];

    ob.scale = scale;
    obs.emplace_back(ob);
  }

  Eigen::Matrix4d T12_initial = T12_.cast<double>();
  std::cout << "T12_initial:\n" << T12_initial << std::endl;

  if (is_tx_fixed_) {
    std::cout << "tx fixed!!!!!" << std::endl;
  }
  algorithm::TwoLasersCalibration(obs, T12_initial, is_tx_fixed_);
  // algorithm::TwoLasersCalibrationAutoDiff(obs, T12_initial);
  // algorithm::TwoLasersCalibrationNaive(obs, T12_initial);

  T12_ = T12_initial.cast<float>();

  return true;
}

void TwoLasersCalib::update_laser2_pose() {
  // 更新激光2的位姿（默认激光1是fix的）
  if (laser_insts_[0].laser_dev_ptr->sensor_id != laser_insts_[1].laser_dev_ptr->sensor_id) {
    // 激光1,2到世界坐标的变换
    Eigen::Matrix4f T_w1, T_w2;
    T_w1 = laser_insts_[0].laser_dev_ptr->get_sensor_pose();

    // 计算激光2的位姿并更新
    T_w2 = T_w1 * T12_;
    laser_insts_[1].laser_dev_ptr->set_sensor_pose(T_w2);
  }
}

void TwoLasersCalib::update_ui_transform() {
  Eigen::Quaternionf q_12(T12_.block<3, 3>(0, 0));
  auto euler = algorithm::quat2euler(q_12.cast<double>());
  transform_12_[0] = T12_(0, 3);
  transform_12_[1] = T12_(1, 3);
  transform_12_[2] = T12_(2, 3);
  transform_12_[3] = RAD2DEG_RBT(euler.roll);
  transform_12_[4] = RAD2DEG_RBT(euler.pitch);
  transform_12_[5] = RAD2DEG_RBT(euler.yaw);
}

void TwoLasersCalib::draw_ui_transform() {
  ImGui::Separator();
  ImGui::Text("T12:");
  ImGui::SameLine();
  ImGui::TextDisabled("the unit: m & deg");
  ImGui::Text("set the transform from laser 2 to 1.");

  // 变更后要更新矩阵
  bool is_changed = false;
  // 设定宽度
  ImGui::PushItemWidth(80);
  if (ImGui::DragScalar("tx  ", ImGuiDataType_Float, &transform_12_[0], 0.1, nullptr, nullptr, "%.3f")) {
    is_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::DragScalar("ty   ", ImGuiDataType_Float, &transform_12_[1], 0.1, nullptr, nullptr, "%.3f")) {
    is_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::DragScalar("tz", ImGuiDataType_Float, &transform_12_[2], 0.1, nullptr, nullptr, "%.3f")) {
    is_changed = true;
  }

  if (ImGui::DragScalar("roll", ImGuiDataType_Float, &transform_12_[3], 1.0, nullptr, nullptr, "%.2f")) {
    is_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::DragScalar("pitch", ImGuiDataType_Float, &transform_12_[4], 1.0, nullptr, nullptr, "%.2f")) {
    is_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::DragScalar("yaw", ImGuiDataType_Float, &transform_12_[5], 1.0, nullptr, nullptr, "%.2f")) {
    is_changed = true;
  }

  ImGui::PopItemWidth();
  // 更新变换矩阵
  if (is_changed) {
    Eigen::Quaterniond q_12 = algorithm::ypr2quaternion(DEG2RAD_RBT(transform_12_[5]), DEG2RAD_RBT(transform_12_[4]),
                                                        DEG2RAD_RBT(transform_12_[3]));
    Eigen::Vector3f t_12{transform_12_[0], transform_12_[1], transform_12_[2]};
    // 更新变换矩阵
    T12_.block<3, 3>(0, 0) = q_12.toRotationMatrix().cast<float>();
    T12_.block<3, 1>(0, 3) = t_12;
    // std::cout << "T12:\n" << T12_ << std::endl;

    update_laser2_pose();
  }

  ImGui::Separator();
}

void TwoLasersCalib::draw_calib_params() {
  const double min_v = 0.;
  ImGui::Separator();
  ImGui::Text("lines detecting params:");

  // 设定宽度
  ImGui::PushItemWidth(80);
  ImGui::DragScalar("max range(m)", ImGuiDataType_Double, &max_range_, 0.1, &min_v, nullptr, "%.2f");
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("set max range for detecting lines.");
  }
  ImGui::SameLine();
  ImGui::DragScalar("angle range(deg)", ImGuiDataType_Double, &angle_range_, 1.0, &min_v, nullptr, "%.1f");
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("set angle range for detecting lines.");
  }

  ImGui::DragScalar("between angle(deg)", ImGuiDataType_Double, &between_angle_, 1.0, &min_v, nullptr, "%.1f");
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("set angle between valid candidate.");
  }

  ImGui::PopItemWidth();
}

void TwoLasersCalib::draw_ui() {
  if (!b_show_window_) {
    return;
  }
  // 新建窗口
  ImGui::Begin("Two Lasers Calibration", &b_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

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
    if (ImGui::Checkbox("show image", &b_show_image_)) {
      if (b_show_image_) {
        laser_insts_[0].img_show_dev_ptr->enable("ts calib laser 1", false);
        laser_insts_[1].img_show_dev_ptr->enable("ts calib laser 2", false);
      } else {
        laser_insts_[0].img_show_dev_ptr->disable();
        laser_insts_[1].img_show_dev_ptr->disable();
      }
    }

    // 闲置状态下才可以设置
    if (next_state_ == STATE_IDLE) {
      draw_calib_params();
    }

    // 设置变换矩阵参数
    draw_ui_transform();

    // 标定逻辑
    calibration();

    if (next_state_ == STATE_IDLE) {
      if (ImGui::Button("start")) {
        b_show_calib_data_ = false;
        // 两个设备名称不能一样
        if (laser_insts_[0].laser_dev_ptr->sensor_id == laser_insts_[1].laser_dev_ptr->sensor_id) {
          std::string msg = "two lasers are the same!";
          dev::show_pfd_info("two lasers calibration", msg);
        } else {
          // 清空上次的标定数据
          // calib_valid_data_vec_.clear();
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
    if (calib_valid_data_vec_.size() > 3) {
      ImGui::SameLine();
      // 开始执行标定
      if (ImGui::Button("calib")) {
        next_state_ = STATE_START_CALIB;
      }
      ImGui::SameLine();
      ImGui::Checkbox("fix_tz", &is_tx_fixed_);
      // tips
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("fix tz value during calibration!");
      }
    }
  }

  // 标定数据相关操作
  if (next_state_ == STATE_IDLE && !calib_valid_data_vec_.empty()) {
    if (ImGui::Checkbox("##show_calib_data", &b_show_calib_data_)) {
      if (b_show_calib_data_) {
        b_need_to_update_cd_ = true;
      }
    }
    if (ImGui::IsItemHovered()) {
      if (b_show_calib_data_) {
        ImGui::SetTooltip("click to not show calib data");
      } else {
        ImGui::SetTooltip("click to show calib data");
      }
    }

    float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
    ImGui::PushButtonRepeat(true);
    ImGui::SameLine(0.0f, spacing);
    if (ImGui::ArrowButton("##left", ImGuiDir_Left)) {
      if (selected_calib_data_id_ > 1) {
        selected_calib_data_id_--;
        if (b_show_calib_data_) {
          b_need_to_update_cd_ = true;
        }
      }
    }
    ImGui::SameLine(0.0f, spacing);
    if (ImGui::ArrowButton("##right", ImGuiDir_Right)) {
      if (selected_calib_data_id_ < calib_valid_data_vec_.size()) {
        selected_calib_data_id_++;
        if (b_show_calib_data_) {
          b_need_to_update_cd_ = true;
        }
      }
    }
    ImGui::PopButtonRepeat();
    ImGui::SameLine();
    // 删除按钮
    if (!calib_valid_data_vec_.empty()) {
      if (ImGui::Button("Del")) {
        selected_calib_data_id_--;
        // 删除数据
        calib_valid_data_vec_.erase(calib_valid_data_vec_.begin() + selected_calib_data_id_);
        b_need_to_update_cd_ = true;
        if (selected_calib_data_id_ == 0) {
          selected_calib_data_id_ = 1;
        }
      }
      // tips
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("delete one item of calib data");
      }
    }

    ImGui::SameLine();
    ImGui::TextDisabled("vaild: %d/%zu", selected_calib_data_id_, calib_valid_data_vec_.size());
  } else {
    ImGui::SameLine();
    ImGui::TextDisabled("vaild: %zu", calib_valid_data_vec_.size());
  }

  ImGui::End();

  // 显示图像
  if (b_show_image_) {
    laser_insts_[0].img_show_dev_ptr->show_image(b_show_image_);
    laser_insts_[1].img_show_dev_ptr->show_image(b_show_image_);
  }
}

}  // namespace calibration
