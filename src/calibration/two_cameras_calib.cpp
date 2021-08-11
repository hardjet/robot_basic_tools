#include <fstream>

#include "imgui.h"
#include "portable-file-dialogs.h"
#include "nlohmann/json.hpp"
#include "cv_bridge_rbt/cv_bridge.h"

#include "glk/glsl_shader.hpp"
#include "glk/primitives/primitives.hpp"

#include "dev/april_board.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "dev/camera.hpp"
#include "dev/util.hpp"

#include "algorithm/two_cameras_ceres.h"
#include "algorithm/util.h"

#include "calibration/task_back_ground.hpp"
#include "calibration/two_cameras_calib.hpp"

#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"
#include "camera_model/camera_models/Camera.h"

#include "util/extrinsics_publisher.hpp"

// ---- 两个相机标定状态
// 空闲
#define STATE_IDLE 0
// 启动
#define STATE_START 1
// 获取相机位姿
#define STATE_GET_POSE 2
// 检查数据稳定性,连续5帧姿态没有大的变化，取中间帧作为候选
#define STATE_CHECK_STEADY 3
// 开始标定
#define STATE_START_CALIB 4
// 在标定过程中
#define STATE_IN_CALIB 5

namespace calibration {

void TwoCamerasCalib::CameraInstType::update() { img_show_dev_ptr->update_image(show_cv_image_ptr); }

TwoCamerasCalib::TwoCamerasCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                                 std::shared_ptr<dev::AprilBoard>& april_board_ptr)
    : BaseCalib(sensor_manager_ptr), april_board_ptr_(april_board_ptr) {
  // 设置id
  camera_insts_[0].id = 0;
  camera_insts_[1].id = 1;

  // 图像显示控件
  camera_insts_[0].img_show_dev_ptr = std::make_shared<dev::ImageShow>();
  camera_insts_[1].img_show_dev_ptr = std::make_shared<dev::ImageShow>();

  // 后台任务
  task_ptr_ = std::make_shared<calibration::Task>();
  // 设置相对位姿初始值
  T_12_ = Eigen::Matrix4f::Identity();

  cur_state_ptr_ = std::make_shared<StateIdle>();
  cur_state_ptr_->set_context(this);
  next_state_ptr_ = std::make_shared<StateIdle>();
  next_state_ptr_->set_context(this);
}

void TwoCamerasCalib::draw_calib_data(glk::GLSLShader& shader) {
  if (!b_show_calib_data_) {
    return;
  }

  auto& cur_calib_data = calib_valid_data_vec_[selected_calib_data_id_ - 1];

  // 更新显示
  if (b_need_to_update_cd_) {
    b_need_to_update_cd_ = false;
    // std::cout << "update calib data" << std::endl;

    // 设置aprilboard在相机1坐标下的位姿
    Eigen::Matrix4f T_c1a = Eigen::Matrix4f::Identity();

    T_c1a.block<3, 3>(0, 0) = cur_calib_data.q_ac[0].toRotationMatrix().transpose().cast<float>();
    T_c1a.block<3, 1>(0, 3) = -T_c1a.block<3, 3>(0, 0) * cur_calib_data.t_ac[0].cast<float>();

    // 计算aprilboard在世界坐标系下的位姿
    Eigen::Matrix4f T_wc1 = camera_insts_[0].camera_dev_ptr->get_sensor_pose();
    Eigen::Matrix4f T_wa = T_wc1 * T_c1a;
    // 设置位姿
    april_board_ptr_->set_pose(T_wa);

    // 没有正常标定计算出的结果
    if (!is_transform_valid_) {
      Eigen::Matrix4f T_ac2 = Eigen::Matrix4f::Identity();
      T_ac2.block<3, 3>(0, 0) = cur_calib_data.q_ac[1].toRotationMatrix().cast<float>();
      T_ac2.block<3, 1>(0, 3) = cur_calib_data.t_ac[1].cast<float>();
      Eigen::Matrix4f T_wc2 = T_wa * T_ac2;
      camera_insts_[1].camera_dev_ptr->set_sensor_pose(T_wc2);

      T_12_ = T_wc1.inverse() * T_wc2;
      // 将计算结果更新到ui
      update_ui_transform();
    }

    // // 需要计算图像点投射在标定板上的点
    // if (cur_calib_data.b_need_calc) {
    //   cur_calib_data.b_need_calc = false;
    //   // 选取标定板上三个不在一条线的点(标定板坐标系下)
    //   std::vector<Eigen::Vector3d> pts_on_april_board(3);
    //   pts_on_april_board[0] = Eigen::Vector3d{0., 0., 0.};
    //   pts_on_april_board[1] = Eigen::Vector3d{0.1, 0.2, 0.};
    //   pts_on_april_board[2] = Eigen::Vector3d{0.2, 0.1, 0.};
    //
    //   // 计算相机坐标系下检测到的角点的空间坐标值
    //   for (uint i = 0; i < 2; i++) {
    //     // std::cout << " --------- " << i << std::endl;
    //     const auto& camera_inst = camera_insts_[i];
    //     // 计算变换矩阵
    //     Eigen::Matrix3d R_ca = cur_calib_data.q_ac[i].toRotationMatrix().transpose();
    //     Eigen::Vector3d t_ca = -R_ca * cur_calib_data.t_ac[i];
    //     // std::cout << "R_ca: \n" << R_ca << std::endl;
    //     // std::cout << "t_ca: \n" << t_ca.transpose() << std::endl;
    //
    //     // 将标定板上三个不在一条线的点变换到相机坐标系下
    //     std::vector<Eigen::Vector3d> pts_on_april_board_in_cam;
    //     // std::cout << "pts_on_april_board_in_cam:" << std::endl;
    //     for (const auto& p : pts_on_april_board) {
    //       pts_on_april_board_in_cam.emplace_back(R_ca * p + t_ca);
    //       // std::cout << pts_on_april_board_in_cam.back().transpose() << std::endl;
    //     }
    //
    //     // 计算平面方程 ax + by + cz + d = 0
    //     Eigen::Vector4d plane_params = algorithm::plane_from_3pts(
    //         pts_on_april_board_in_cam[0], pts_on_april_board_in_cam[1], pts_on_april_board_in_cam[2]);
    //
    //     // std::cout << "plane_params: " << plane_params.transpose() << std::endl;
    //     // 检查是否在平面上
    //     // std::cout << "check on plane: " << std::endl;
    //     // std::cout << plane_params.head(3).transpose() * pts_on_april_board_in_cam[0] + plane_params[3] <<
    //     std::endl;
    //     // std::cout << plane_params.head(3).transpose() * pts_on_april_board_in_cam[1] + plane_params[3] <<
    //     std::endl;
    //     // std::cout << plane_params.head(3).transpose() * pts_on_april_board_in_cam[2] + plane_params[3] <<
    //     std::endl;
    //
    //     // 计算图像上落在标定板上的点
    //     for (const auto& p : cur_calib_data.image_points[i]) {
    //       Eigen::Vector3d p_in_space;
    //       // std::cout << "img:" << p.x << ", " << p.y << std::endl;
    //       camera_inst.camera_dev_ptr->camera_model()->liftProjective(Eigen::Vector2d{p.x, p.y}, p_in_space);
    //       // std::cout << "p_in_space: " << p_in_space.transpose() << std::endl;
    //       Eigen::Vector3d p_on_board;
    //       // 计算交点
    //       bool res = algorithm::plane_line_intersect_point(plane_params.head(3), pts_on_april_board_in_cam[1],
    //                                                        p_in_space, p_in_space, p_on_board);
    //       if (res) {
    //         cur_calib_data.pts_on_board[i].emplace_back(p_on_board);
    //         // std::cout << "p_on_board: " << p_on_board.transpose() << std::endl;
    //         // std::cout << "check on plane:" << plane_params.head(3).transpose() * p_on_board + plane_params[3]
    //         //           << std::endl;
    //       }
    //     }
    //   }
    // }

    // 需要计算图像点投射在标定板上的点
    if (cur_calib_data.b_need_calc) {
      cur_calib_data.b_need_calc = false;
      // 选取标定板上三个不在一条线的点(标定板坐标系下)
      std::vector<Eigen::Vector3d> pts_on_april_board(3);
      pts_on_april_board[0] = Eigen::Vector3d{0., 0., 0.};
      pts_on_april_board[1] = Eigen::Vector3d{0.1, 0.2, 0.};
      pts_on_april_board[2] = Eigen::Vector3d{0.2, 0.1, 0.};

      // 计算相机坐标系下检测到的角点的空间坐标值
      for (uint i = 0; i < 2; i++) {
        // std::cout << " --------- " << i << std::endl;
        const auto& camera_inst = camera_insts_[i];
        Eigen::Matrix3d R_ca;
        Eigen::Vector3d t_ca;
        // 计算变换矩阵
        if (i == 0) {
          R_ca = cur_calib_data.q_ac[i].toRotationMatrix().transpose();
          t_ca = -R_ca * cur_calib_data.t_ac[i];
        } else {
          // 相机2使用T_12计算
          // Eigen::Matrix3d R_c1a = cur_calib_data.q_ac[0].toRotationMatrix().transpose();
          // Eigen::Vector3d t_c1a = -R_ca * cur_calib_data.t_ac[0];
          Eigen::Matrix4f T_21 = T_12_.inverse();
          Eigen::Matrix4f T_c2a = T_21 * T_c1a;
          R_ca = T_c2a.block<3, 3>(0, 0).cast<double>();
          t_ca = T_c2a.block<3, 1>(0, 3).cast<double>();
        }
        // std::cout << "R_ca: \n" << R_ca << std::endl;
        // std::cout << "t_ca: \n" << t_ca.transpose() << std::endl;

        // 将标定板上三个不在一条线的点变换到相机坐标系下
        std::vector<Eigen::Vector3d> pts_on_april_board_in_cam;
        // std::cout << "pts_on_april_board_in_cam:" << std::endl;
        for (const auto& p : pts_on_april_board) {
          pts_on_april_board_in_cam.emplace_back(R_ca * p + t_ca);
          // std::cout << pts_on_april_board_in_cam.back().transpose() << std::endl;
        }

        // 计算平面方程 ax + by + cz + d = 0
        Eigen::Vector4d plane_params = algorithm::plane_from_3pts(
            pts_on_april_board_in_cam[0], pts_on_april_board_in_cam[1], pts_on_april_board_in_cam[2]);

        // std::cout << "plane_params: " << plane_params.transpose() << std::endl;
        // 检查是否在平面上
        // std::cout << "check on plane: " << std::endl;
        // std::cout << plane_params.head(3).transpose() * pts_on_april_board_in_cam[0] + plane_params[3] << std::endl;
        // std::cout << plane_params.head(3).transpose() * pts_on_april_board_in_cam[1] + plane_params[3] << std::endl;
        // std::cout << plane_params.head(3).transpose() * pts_on_april_board_in_cam[2] + plane_params[3] << std::endl;

        // 计算图像上落在标定板上的点
        for (const auto& p : cur_calib_data.image_points[i]) {
          Eigen::Vector3d p_in_space;
          // std::cout << "img:" << p.x << ", " << p.y << std::endl;
          camera_inst.camera_dev_ptr->camera_model()->liftProjective(Eigen::Vector2d{p.x, p.y}, p_in_space);
          // std::cout << "p_in_space: " << p_in_space.transpose() << std::endl;
          Eigen::Vector3d p_on_board;
          // 计算交点
          bool res = algorithm::plane_line_intersect_point(plane_params.head(3), pts_on_april_board_in_cam[1],
                                                           p_in_space, p_in_space, p_on_board);
          if (res) {
            cur_calib_data.pts_on_board[i].emplace_back(p_on_board);
            // std::cout << "p_on_board: " << p_on_board.transpose() << std::endl;
            // std::cout << "check on plane:" << plane_params.head(3).transpose() * p_on_board + plane_params[3]
            //           << std::endl;
          }
        }
      }
    }
  }

  // 画点
  auto draw_points = [&shader](Eigen::Matrix4f model, const Eigen::Vector3d& pos) {
    model.block<3, 1>(0, 3) = model.block<3, 3>(0, 0) * pos.cast<float>() + model.block<3, 1>(0, 3);

    // 更改大小 设置球的半径大小
    model.block<3, 3>(0, 0) *= 0.001;
    // 改变位置
    shader.set_uniform("model_matrix", model);

    const auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    sphere.draw(shader);
  };

  shader.set_uniform("color_mode", 1);
  if (!cur_calib_data.b_need_calc) {
    for (uint i = 0; i < 2; i++) {
      // 设置显示颜色
      if (i == 0) {
        shader.set_uniform("material_color", Eigen::Vector4f(1.f, 0.f, 0.f, 1.0f));
      } else {
        shader.set_uniform("material_color", Eigen::Vector4f(0.f, 1.f, 0.f, 1.0f));
      }

      // 显示点
      for (const auto& pt : cur_calib_data.pts_on_board[i]) {
        draw_points(camera_insts_[i].camera_dev_ptr->get_sensor_pose(), pt);
      }
    }
  }
}

void TwoCamerasCalib::draw_gl(glk::GLSLShader& shader) {
  if (!b_show_window_) {
    return;
  }

  draw_calib_data(shader);
}

void TwoCamerasCalib::update_related_pose() {
  // 相机1到世界坐标的变换
  Eigen::Matrix4f T_w1;
  T_w1 = camera_insts_[0].camera_dev_ptr->get_sensor_pose();

  // 计算相机2的位姿并更新
  Eigen::Matrix4f T_w2 = T_w1 * T_12_;
  camera_insts_[1].camera_dev_ptr->set_sensor_pose(T_w2);

  if (b_show_calib_data_) {
    b_need_to_update_cd_ = true;
  }
}

void TwoCamerasCalib::update_ui_transform() {
  Eigen::Quaternionf q_12(T_12_.block<3, 3>(0, 0));
  auto euler = algorithm::quat2euler(q_12.cast<double>());
  transform_12_[0] = T_12_(0, 3);
  transform_12_[1] = T_12_(1, 3);
  transform_12_[2] = T_12_(2, 3);
  transform_12_[3] = RAD2DEG_RBT(euler.roll);
  transform_12_[4] = RAD2DEG_RBT(euler.pitch);
  transform_12_[5] = RAD2DEG_RBT(euler.yaw);
}

void TwoCamerasCalib::update() {
  std::lock_guard<std::mutex> lock(mtx_);

  // 更新数据
  for (auto& camera_inst : camera_insts_) {
    if (!camera_inst.cv_image_data_ptr) {
      camera_inst.cv_image_data_ptr = camera_inst.camera_dev_ptr->data();
      if (camera_inst.cv_image_data_ptr) {
        camera_inst.is_new_data = true;
      }
    } else {
      auto camera_data = camera_inst.camera_dev_ptr->data();
      if (camera_inst.cv_image_data_ptr->header.stamp.nsec != camera_data->header.stamp.nsec) {
        camera_inst.cv_image_data_ptr = camera_data;
        camera_inst.is_new_data = true;
      }
    }
  }
}

bool TwoCamerasCalib::get_valid_pose() {
  // 当前处理的图像数据
  std::vector<boost::shared_ptr<const cv_bridge::CvImage>> cur_images(2);

  // 将图像锁定
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (int i = 0; i < 2; i++) {
      auto& cv_image_ptr = camera_insts_[i].cv_image_data_ptr;
      camera_insts_[i].frame_id = cv_image_ptr->header.frame_id;
      printf("----- TwoCamerasCalib::get_valid_pose() ..... frame id = %s\n", cv_image_ptr->header.frame_id.c_str());
      cur_images[i].reset(
          new const cv_bridge::CvImage(cv_image_ptr->header, cv_image_ptr->encoding, cv_image_ptr->image));
    }
  }

  // 相机坐标系到aprilboard坐标系的变换
  Eigen::Matrix4d T_ac = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 2; i++) {
    auto& camera_inst = camera_insts_[i];
    cv::Mat img;
    // 需要转为灰度
    if (cur_images[i]->image.channels() == 3) {
      cv::cvtColor(cur_images[i]->image, img, cv::COLOR_RGB2GRAY);
    } else {
      img = cur_images[i]->image.clone();
    }

    cv::Mat img_show;
    // 需要转为彩色图像
    if (cur_images[i]->image.channels() == 1) {
      cv::cvtColor(cur_images[i]->image, img_show, cv::COLOR_GRAY2RGB);
    } else {
      img_show = cur_images[i]->image.clone();
    }

    if (april_board_ptr_->board->computeObservation(img, img_show, calib_data_.object_points[i],
                                                    calib_data_.image_points[i])) {
      // 计算外参T_ac camera_model->aprilboard
      camera_inst.camera_dev_ptr->camera_model()->estimateExtrinsics(calib_data_.object_points[i],
                                                                     calib_data_.image_points[i], T_ac, img_show);
      calib_data_.timestamp[i] = cur_images[i]->header.stamp.toSec();
      calib_data_.t_ac[i] = T_ac.block(0, 3, 3, 1);
      Eigen::Matrix3d R_ac = T_ac.block(0, 0, 3, 3);
      Eigen::Quaterniond q_ac(R_ac);
      calib_data_.q_ac[i] = q_ac;

      // 保存
      camera_inst.show_cv_image_ptr.reset(
          new const cv_bridge::CvImage(cur_images[i]->header, cur_images[i]->encoding, img_show));
      // 更新显示图象
      camera_inst.update();
      // std::cout << i << " ok!" << std::endl;
    } else {
      // std::cout << "no april board detected!" << std::endl;
      return false;
    }
  }

  return true;
}

void TwoCamerasCalib::check_and_save() {
  bool b_need_to_save = true;
  // 逐个检测角度值
  for (auto& data : calib_valid_data_vec_) {
    double theta = 2 * std::acos((data.q_ac[0].inverse() * calib_data_vec_.at(2).q_ac[0]).w());
    // 保证间隔x以上
    if (theta < DEG2RAD_RBT(between_angle_)) {
      b_need_to_save = false;
      break;
    }
  }

  if (b_need_to_save) {
    // 取第3帧保存
    calib_valid_data_vec_.push_back(calib_data_vec_.at(2));
    is_transform_valid_ = false;
    clear_calib_data_flag();
    printf("!!!!!!!!!!!!!tz: %f saved!\n", calib_valid_data_vec_.back().t_ac[0].z());
  }
}

// 使用平均值的方法的计算(效果不好)
// bool TwoCamerasCalib::calc() {
//   Eigen::Matrix4d T_ac1 = Eigen::Matrix4d::Identity();
//   Eigen::Matrix4d T_ac2 = Eigen::Matrix4d::Identity();
//   Eigen::Vector3d t_12{0, 0, 0};
//   Eigen::Vector3d euler_12{0, 0, 0};
//
//   // 计算平均位姿
//   for (const auto& data : calib_valid_data_vec_) {
//     // 计算camera 2 到 1的位姿变换
//     T_ac1.block<3, 3>(0, 0) = data.q_ac[0].toRotationMatrix();
//     T_ac1.block<3, 1>(0, 3) = data.t_ac[0];
//     T_ac2.block<3, 3>(0, 0) = data.q_ac[1].toRotationMatrix();
//     T_ac2.block<3, 1>(0, 3) = data.t_ac[1];
//
//     Eigen::Matrix4d T_12 = T_ac1.inverse() * T_ac2;
//     // std::cout << "t_12:" << T_12.block<3, 1>(0, 3).transpose() << std::endl;
//
//     t_12 += T_12.block<3, 1>(0, 3);
//
//     auto euler = algorithm::quat2euler(Eigen::Quaterniond(T_12.block<3, 3>(0, 0)));
//     // std::cout << "euler_12: " << euler << std::endl;
//     euler_12.x() += euler.roll;
//     euler_12.y() += euler.pitch;
//     euler_12.z() += euler.yaw;
//   }
//
//   t_12 /= double(calib_valid_data_vec_.size());
//   euler_12 /= double(calib_valid_data_vec_.size());
//   // std::cout << "final t_12: " << t_12.transpose() << std::endl;
//   // std::cout << "final euler_12(rpy): " << euler_12.transpose() * (180 / M_PI) << std::endl;
//
//   // 更新变换矩阵
//   T_12_.block<3, 1>(0, 3) = t_12.cast<float>();
//   T_12_.block<3, 3>(0, 0) =
//       algorithm::ypr2quat(euler_12.z(), euler_12.y(), euler_12.x()).toRotationMatrix().cast<float>();
//   is_transform_valid_ = true;
//
//   return true;
// }

bool TwoCamerasCalib::calc() {
  Eigen::Matrix4d T_ac1 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_ac2 = Eigen::Matrix4d::Identity();
  Eigen::Vector3d t_12{0, 0, 0};
  Eigen::Vector3d euler_12{0, 0, 0};

  // 选取标定数据中的一个值作为初值
  // 计算camera 2 到 1的位姿变换
  T_ac1.block<3, 3>(0, 0) = calib_valid_data_vec_[0].q_ac[0].toRotationMatrix();
  T_ac1.block<3, 1>(0, 3) = calib_valid_data_vec_[0].t_ac[0];
  T_ac2.block<3, 3>(0, 0) = calib_valid_data_vec_[0].q_ac[1].toRotationMatrix();
  T_ac2.block<3, 1>(0, 3) = calib_valid_data_vec_[0].t_ac[1];
  Eigen::Matrix4d T_12_init = T_ac1.inverse() * T_ac2;

  // 观测数据
  algorithm::TwoCamerasCalib::Observation obs;
  for (const auto& data : calib_valid_data_vec_) {
    // c1->aprilboard
    Eigen::Matrix3d R_ac1 = data.q_ac[0].toRotationMatrix();
    Eigen::Vector3d t_ac1 = data.t_ac[0];

    // aprilboard -> c1
    Eigen::Matrix3d R_c1a = R_ac1.transpose();
    Eigen::Vector3d t_c1a = -R_c1a * t_ac1;

    // 将相机2观测到的aprilboard坐标系下的点 变换到 相机1坐标系下
    for (uint i = 0; i < data.object_points[1].size(); i++) {
      const auto& pt = data.object_points[1].at(i);
      Eigen::Vector3d pt_in_cam1 = R_c1a * Eigen::Vector3d(pt.x, pt.y, pt.z) + t_c1a;
      // std::cout << "obj in april: " << Eigen::Vector3d(pt.x, pt.y, pt.z).transpose() << std::endl;
      obs.object_points.emplace_back(pt_in_cam1);
      // std::cout << "obj in cam1: " << obs.object_points.back().transpose() << std::endl;
      obs.image_points.emplace_back(Eigen::Vector2d(data.image_points[1].at(i).x, data.image_points[1].at(i).y));
      // std::cout << "pt in cam2: " << obs.image_points.back().transpose() << std::endl;
    }
  }

  Eigen::Matrix4d T_21 = T_12_init.inverse();
  bool res = algorithm::TwoCamerasCalib::calibrate(camera_insts_[1].camera_dev_ptr->camera_model(), obs, T_21);

  if (res) {
    // 更新变换矩阵
    T_12_ = T_21.inverse().cast<float>();
    return true;
  }

  return false;
}

void TwoCamerasCalib::clear_calib_data_flag() {
  for (auto& data : calib_valid_data_vec_) {
    data.b_need_calc = true;
    data.pts_on_board[0].clear();
    data.pts_on_board[1].clear();
  }
}

void TwoCamerasCalib::change_current_state(std::shared_ptr<CalibrationState> new_state) {
  cur_state_ptr_ = std::move(new_state);
  cur_state_ptr_->set_context(this);
}

void TwoCamerasCalib::change_next_state(std::shared_ptr<CalibrationState> new_state) {
  next_state_ptr_ = std::move(new_state);
  next_state_ptr_->set_context(this);
}

bool TwoCamerasCalib::instrument_available() {
  if (camera_insts_[0].is_new_data && camera_insts_[1].is_new_data) {
    camera_insts_[0].is_new_data = false;
    camera_insts_[1].is_new_data = false;
    return true;
  } else {
    return false;
  }
}

int TwoCamerasCalib::pose_valid() {
  if (task_ptr_->do_task("get_valid_pose", std::bind(&TwoCamerasCalib::get_valid_pose, this))) {  // NOLINT
    if (task_ptr_->result<bool>()) {
      return 1;
    } else {
      return 2;
    }
  } else {
    return 3;
  }
}

void TwoCamerasCalib::check_steady() {
  if (calib_data_vec_.empty()) {
    calib_data_vec_.push_back(calib_data_);
  } else {
    // 检查相机位姿是否一致
    double dist = (calib_data_vec_.at(0).t_ac[0] - calib_data_.t_ac[0]).norm();
    // 四元数的转角是原本的1/2
    double theta = 2 * std::acos((calib_data_vec_.at(0).q_ac[0].inverse() * calib_data_.q_ac[0]).w());
    std::cout << "dist:" << dist << ", theta:" << RAD2DEG_RBT(theta) << std::endl;
    // 抖动小于1cm与0.5°
    if (dist < jitter_dist_ && theta < DEG2RAD_RBT(jitter_angle_)) {
      calib_data_vec_.push_back(calib_data_);
      // 足够稳定才保存
      if (calib_data_vec_.size() > 3) {
        check_and_save();
      }
    } else {
      // 抖动大则重新开始检测
      calib_data_vec_.clear();
      calib_data_vec_.push_back(calib_data_);
      std::cout << "moved!!!" << std::endl;
    }
  }
}

int TwoCamerasCalib::do_calib() {
  if (task_ptr_->do_task("calc", std::bind(&TwoCamerasCalib::calc, this))) {  // NOLINT
    // 结束后需要读取结果
    if (task_ptr_->result<bool>()) {
      // 将计算结果更新到相机2
      update_related_pose();
      update_ui_transform();
      is_transform_valid_ = true;
      clear_calib_data_flag();
      return 1;
    } else {
      return 2;
    }
  } else {
    return 3;
  }
}

void TwoCamerasCalib::new_calibration() {
  update();
  cur_state_ptr_->calibration();
}

void TwoCamerasCalib::calibration() {
  // 首先获取最新的数据
  update();

  switch (cur_state_) {
    case STATE_IDLE:
      cur_state_ = next_state_;
      break;
    case STATE_START:
      // 都有新数据才计算
      if (camera_insts_[0].is_new_data && camera_insts_[1].is_new_data) {
        camera_insts_[0].is_new_data = false;
        camera_insts_[1].is_new_data = false;
        cur_state_ = STATE_GET_POSE;
      } else {
        cur_state_ = STATE_IDLE;
      }
      break;
    case STATE_GET_POSE:
      // 执行任务
      if (task_ptr_->do_task("get_valid_pose", std::bind(&TwoCamerasCalib::get_valid_pose, this))) {  // NOLINT
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
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
        double dist = (calib_data_vec_.at(0).t_ac[0] - calib_data_.t_ac[0]).norm();
        // 四元数的转角是原本的1/2
        double theta = 2 * std::acos((calib_data_vec_.at(0).q_ac[0].inverse() * calib_data_.q_ac[0]).w());
        std::cout << "dist:" << dist << ", theta:" << RAD2DEG_RBT(theta) << std::endl;
        // 抖动小于1cm与0.5°
        if (dist < jitter_dist_ && theta < DEG2RAD_RBT(jitter_angle_)) {
          calib_data_vec_.push_back(calib_data_);
          // 足够稳定才保存
          if (calib_data_vec_.size() > 3) {
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
      if (task_ptr_->do_task("calc", std::bind(&TwoCamerasCalib::calc, this))) {  // NOLINT
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          // 将计算结果更新到相机2
          update_related_pose();
          update_ui_transform();
          is_transform_valid_ = true;
          clear_calib_data_flag();
          cur_state_ = STATE_IDLE;
        } else {
          std::cout << "calibration failed!!!" << std::endl;
        }
      }
      break;
  }
}

static void convert_json_to_pts(std::array<std::vector<cv::Point2f>, 2>& imgs_pts, nlohmann::json& js) {
  // 解包数据
  std::vector<std::vector<std::vector<float>>> v;
  js.get_to<std::vector<std::vector<std::vector<float>>>>(v);

  for (int i = 0; i < 2; i++) {
    imgs_pts[i].resize(v[i].size());
    for (uint j = 0; j < imgs_pts[i].size(); j++) {
      imgs_pts[i].at(j) = cv::Point2f{v[i][j][0], v[i][j][1]};
    }
  }
}

static void convert_json_to_pts(std::array<std::vector<cv::Point3f>, 2>& objs_pts, nlohmann::json& js) {
  // 解包数据
  std::vector<std::vector<std::vector<float>>> v;
  js.get_to<std::vector<std::vector<std::vector<float>>>>(v);

  for (int i = 0; i < 2; i++) {
    objs_pts[i].resize(v[i].size());
    for (uint j = 0; j < objs_pts[i].size(); j++) {
      objs_pts[i].at(j) = cv::Point3f{v[i][j][0], v[i][j][1], v[i][j][2]};
    }
  }
}

// static void convert_json_to_pts(std::array<std::vector<cv::Point2f>, 2>& imgs_pts, nlohmann::json& js) {
//   std::cout << "image_points" << std::endl;
//   for (int i = 0; i < 2; i++) {
//     imgs_pts[i].resize(js.at(i).size());
//     for (uint j = 0; j < imgs_pts[i].size(); j++) {
//       imgs_pts[i].at(j) = cv::Point2f{js.at(i).at(j).at(0).get<float>(), js.at(i).at(j).at(1).get<float>()};
//       std::cout << imgs_pts[i].at(j).x << ", " << imgs_pts[i].at(j).y << std::endl;
//     }
//   }
// }
//
// static void convert_json_to_pts(std::array<std::vector<cv::Point3f>, 2>& objs_pts, nlohmann::json& js) {
//   std::cout << "object_points" << std::endl;
//   for (int i = 0; i < 2; i++) {
//     objs_pts[i].resize(js.at(i).size());
//     for (uint j = 0; j < objs_pts[i].size(); j++) {
//       objs_pts[i].at(j) = cv::Point3f{js.at(i).at(j).at(0).get<float>(), js.at(i).at(j).at(1).get<float>(),
//                                       js.at(i).at(j).at(2).get<float>()};
//       std::cout << objs_pts[i].at(j).x << ", " << objs_pts[i].at(j).y << ", " << objs_pts[i].at(j).z << std::endl;
//     }
//   }
// }

bool TwoCamerasCalib::load_calib_data(const std::string& file_path) {
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

  if (!js_whole.contains("type") || js_whole["type"] != "two_cameras_calibration") {
    std::cout << "wrong file type!" << std::endl;
    return false;
  }

  // 清空之前的数据
  calib_valid_data_vec_.clear();
  is_transform_valid_ = false;

  // 加载数据
  for (const auto& data : js_whole["data"].items()) {
    CalibData d;
    std::vector<double> v;
    data.value().at("timestamp").get_to<std::vector<double>>(v);
    d.timestamp[0] = v[0];
    d.timestamp[1] = v[1];
    v.clear();
    data.value().at("q_ac_0").get_to<std::vector<double>>(v);
    d.q_ac[0].x() = v[0];
    d.q_ac[0].y() = v[1];
    d.q_ac[0].z() = v[2];
    d.q_ac[0].w() = v[3];
    v.clear();
    data.value().at("q_ac_1").get_to<std::vector<double>>(v);
    d.q_ac[1].x() = v[0];
    d.q_ac[1].y() = v[1];
    d.q_ac[1].z() = v[2];
    d.q_ac[1].w() = v[3];
    v.clear();
    data.value().at("t_ac_0").get_to<std::vector<double>>(v);
    d.t_ac[0] = Eigen::Map<Eigen::VectorXd>(v.data(), 3);
    v.clear();
    data.value().at("t_ac_1").get_to<std::vector<double>>(v);
    d.t_ac[1] = Eigen::Map<Eigen::VectorXd>(v.data(), 3);
    v.clear();

    // 加载检测到的aprilboard图像角点
    convert_json_to_pts(d.image_points, data.value().at("image_points"));
    // 加载图像角点对应的aprilboard坐标系下的空间点
    convert_json_to_pts(d.object_points, data.value().at("object_points"));
    calib_valid_data_vec_.push_back(d);
  }

  for (const auto& data : js_whole["frame"].items()) {
    if (std::strcmp(data.key().c_str(), "from_frame") == 0) {
      camera_insts_[1].frame_id = data.value();
    } else {
      camera_insts_[0].frame_id = data.value();
    }
  }
  return true;
}

static nlohmann::json convert_pts_to_json(const std::array<std::vector<cv::Point2f>, 2>& imgs_pts) {
  std::vector<nlohmann::json> json_imgs_pts;

  for (const auto& img_pts : imgs_pts) {
    std::vector<nlohmann::json> json_pts(img_pts.size());
    for (unsigned int idx = 0; idx < img_pts.size(); ++idx) {
      json_pts.at(idx) = {img_pts.at(idx).x, img_pts.at(idx).y};
    }
    json_imgs_pts.emplace_back(json_pts);
  }

  return json_imgs_pts;
}

static nlohmann::json convert_pts_to_json(const std::array<std::vector<cv::Point3f>, 2>& objs_pts) {
  std::vector<nlohmann::json> json_objs_pts;

  for (const auto& obj_pts : objs_pts) {
    std::vector<nlohmann::json> json_pts(obj_pts.size());
    for (unsigned int idx = 0; idx < obj_pts.size(); ++idx) {
      json_pts.at(idx) = {obj_pts.at(idx).x, obj_pts.at(idx).y, obj_pts.at(idx).z};
    }
    json_objs_pts.emplace_back(json_pts);
  }

  return json_objs_pts;
}

bool TwoCamerasCalib::save_extrinsics(const std::string& file_path) {
  if (transform_12_.empty()) {
    return false;
  }

  nlohmann::json js_file;
  nlohmann::json js_data;
  nlohmann::json js_frame;

  js_data = {{"tx", transform_12_[0]},   {"ty", transform_12_[1]},    {"tz", transform_12_[2]},
             {"roll", transform_12_[3]}, {"pitch", transform_12_[4]}, {"yaw", transform_12_[5]}};
  js_frame = {{"from_frame", camera_insts_[1].frame_id}, {"to_frame", camera_insts_[0].frame_id}};

  js_file["data"] = js_data;
  js_file["frame"] = js_frame;
  js_file["type"] = "two_cameras_extrinsics";

  std::ofstream ofs(file_path, std::ios::out);
  if (ofs.is_open()) {
    std::cout << "save data to " << file_path.c_str() << std::endl;
    ofs << std::setw(2) << js_file << std::endl;
    ofs.close();
    return true;
  } else {
    std::cout << "cannot create a file at " << file_path.c_str() << std::endl;
    return false;
  }
}

bool TwoCamerasCalib::save_calib_data(const std::string& file_path) {
  if (calib_valid_data_vec_.empty()) {
    return false;
  }

  nlohmann::json js_whole;
  js_whole["type"] = "two_cameras_calibration";

  // 整理数据
  std::vector<nlohmann::json> js_data;
  for (const auto& data : calib_valid_data_vec_) {
    nlohmann::json js = {{"timestamp", {data.timestamp[0], data.timestamp[1]}},
                         {"q_ac_0", {data.q_ac[0].x(), data.q_ac[0].y(), data.q_ac[0].z(), data.q_ac[0].w()}},
                         {"q_ac_1", {data.q_ac[1].x(), data.q_ac[1].y(), data.q_ac[1].z(), data.q_ac[1].w()}},
                         {"t_ac_0", {data.t_ac[0].x(), data.t_ac[0].y(), data.t_ac[0].z()}},
                         {"t_ac_1", {data.t_ac[1].x(), data.t_ac[1].y(), data.t_ac[1].z()}},
                         {"image_points", convert_pts_to_json(data.image_points)},
                         {"object_points", convert_pts_to_json(data.object_points)}};
    js_data.push_back(js);
  }
  nlohmann::json js_frame = {{"from_frame", camera_insts_[1].frame_id}, {"to_frame", camera_insts_[0].frame_id}};

  js_whole["data"] = js_data;
  js_whole["frame"] = js_frame;

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

void TwoCamerasCalib::draw_calib_params() {
  const double min_v = 0.;
  ImGui::Separator();
  ImGui::Text("calibration params:");

  ImGui::PushItemWidth(80);
  ImGui::DragScalar("jitter dist thd(m)", ImGuiDataType_Double, &jitter_dist_, 1.0, &min_v, nullptr, "%.3f");
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("within dist thd for steady data.");
  }

  ImGui::DragScalar("jitter angle thd(deg)", ImGuiDataType_Double, &jitter_angle_, 1.0, &min_v, nullptr, "%.2f");
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("within angle thd for steady data.");
  }

  ImGui::DragScalar("between angle(deg)", ImGuiDataType_Double, &between_angle_, 1.0, &min_v, nullptr, "%.1f");
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("set angle between valid candidate.");
  }

  ImGui::PopItemWidth();
  ImGui::Separator();
}

void TwoCamerasCalib::draw_ui_transform() {
  ImGui::Text("T_12:");
  ImGui::SameLine();
  ImGui::TextDisabled("the unit: m & deg");
  //  ImGui::Text("set the transform from camera %s to 1.", camera_insts_[1].camera_dev_ptr->sensor_name.c_str(), );
  ImGui::Text("set the transform from camera %s to %s.", camera_insts_[1].camera_dev_ptr->sensor_name.c_str(),
              camera_insts_[0].camera_dev_ptr->sensor_name.c_str());

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
    Eigen::Quaterniond q_12 = algorithm::ypr2quat(DEG2RAD_RBT(transform_12_[5]), DEG2RAD_RBT(transform_12_[4]),
                                                  DEG2RAD_RBT(transform_12_[3]));
    Eigen::Vector3f t_12{transform_12_[0], transform_12_[1], transform_12_[2]};
    // 更新变换矩阵
    T_12_.block<3, 3>(0, 0) = q_12.toRotationMatrix().cast<float>();
    T_12_.block<3, 1>(0, 3) = t_12;
    // std::cout << "T12:\n" << T_12_ << std::endl;
    update_related_pose();
  }

  ImGui::Separator();
}

void TwoCamerasCalib::pass_nh(const ros::NodeHandle& nh) { ros_nh_ = nh; }

void TwoCamerasCalib::draw_ui() {
  if (!b_show_window_) {
    return;
  }

  // 新建窗口
  ImGui::Begin("Two Cameras Calibration", &b_show_window_, ImGuiWindowFlags_AlwaysAutoResize);

  // 相机选择 - 相机 1
  draw_sensor_selector<dev::Camera>("camera 1  ", dev::CAMERA, camera_insts_[0].camera_dev_ptr);
  // 下拉菜单选择相机1的base frame
  static int item_current_idx = 0;
  const char* combo_preview_value = major_frames_list_[item_current_idx].c_str();
  ImGui::Text("base frame:");
  ImGui::SameLine();
  if (ImGui::BeginCombo("##selected frame 1", combo_preview_value, ImGuiComboFlags_None)) {
    for (int n = 0; n < major_frames_list_.size(); n++) {
      const bool is_selected = (item_current_idx == n);
      if (ImGui::Selectable(major_frames_list_[n].c_str(), is_selected)) {
        item_current_idx = n;
      }
      if (is_selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  // 显示第一个相机图像流的true frame
  ImGui::Text("true frame: %s\n", camera_insts_[0].frame_id.c_str());

  ImGui::Separator();

  // 相机选择 - 相机 2
  draw_sensor_selector<dev::Camera>("camera 2  ", dev::CAMERA, camera_insts_[1].camera_dev_ptr);
  // 下拉菜单选择第二个相机的base frame
  static int item_current_idx2 = 0;
  const char* combo_preview_value2 = major_frames_list_[item_current_idx2].c_str();
  ImGui::Text("base frame:");
  ImGui::SameLine();
  if (ImGui::BeginCombo("##selected frame 2", combo_preview_value2, ImGuiComboFlags_None)) {
    for (int nn = 0; nn < major_frames_list_.size(); nn++) {
      const bool is_selected2 = (item_current_idx2 == nn);
      if (ImGui::Selectable(major_frames_list_[nn].c_str(), is_selected2)) {
        item_current_idx2 = nn;
      }
      if (is_selected2) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  // 显示第二个相机的true frame
  ImGui::Text("true frame: %s\n", camera_insts_[1].frame_id.c_str());

  // 显示读取参数按钮 R
  if (camera_insts_[0].camera_dev_ptr) {
    // 从文件加载标定数据
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
      ImGui::SetTooltip("load calib data from .json file");
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
  }

  // 设备就绪后才能标定
  if (camera_insts_[0].camera_dev_ptr && camera_insts_[1].camera_dev_ptr) {
    ImGui::SameLine();
    ImGui::Dummy(ImVec2(200, 5));
    ImGui::SameLine();
    // 选择是否显示图像
    if (ImGui::Checkbox("show image", &b_show_image_)) {
      if (b_show_image_) {
        camera_insts_[0].img_show_dev_ptr->enable("tc calib: camera 1", false);
        camera_insts_[1].img_show_dev_ptr->enable("tc calib: camera 2", false);
      } else {
        camera_insts_[0].img_show_dev_ptr->disable();
        camera_insts_[1].img_show_dev_ptr->disable();
      }
    }

    // 闲置状态下才可以设置
    //    if (next_state_ == STATE_IDLE) {
    //      draw_calib_params();
    //    }
    if (next_state_ptr_->id() == 0) {
      draw_calib_params();
    }

    // 设置变换矩阵参数
    draw_ui_transform();

    // 标定逻辑
    //    calibration();
    new_calibration();

    //    if (next_state_ == STATE_IDLE) {
    if (next_state_ptr_->id() == 0) {
      if (ImGui::Button("start")) {
        // 两个设备名称不能一样，两个被选择的base frame不能一样
        if ((camera_insts_[0].camera_dev_ptr->sensor_id == camera_insts_[1].camera_dev_ptr->sensor_id) ||
            std::strcmp(major_frames_list_[item_current_idx].c_str(), major_frames_list_[item_current_idx2].c_str()) ==
                0) {
          std::string msg = "two cameras are the same!";
          dev::show_pfd_info("two cameras calibration", msg);
        } else if (!camera_insts_[0].camera_dev_ptr->camera_model() ||
                   !camera_insts_[1].camera_dev_ptr->camera_model()) {
          // 检测相机模型是否已经选择
          std::string msg = "please set camera model first!";
          dev::show_pfd_info("info", msg);
        } else {
          b_show_calib_data_ = false;
          // 清空上次的标定数据
          // calib_valid_data_vec_.clear();
          //          next_state_ = STATE_START;
          change_next_state(std::make_shared<StateStart>());
        }
      }
    } else {
      if (ImGui::Button("stop")) {
        //        next_state_ = STATE_IDLE;
        change_next_state(std::make_shared<StateIdle>());
      }
    }

    // 标定状态只需要设定一次
    //    if (cur_state_ == STATE_IN_CALIB) {
    //      next_state_ = STATE_IDLE;
    //    }
    if (cur_state_ptr_->id() == 5) {
      change_next_state(std::make_shared<StateIdle>());
    }

    // 有一帧数据就可以开始进行标定操作了
    if (!calib_valid_data_vec_.empty()) {
      ImGui::SameLine();
      // 开始执行标定
      if (ImGui::Button("calib")) {
        // 两个设备名称不能一样 ，两个被选择的base frame不能一样
        if ((camera_insts_[0].camera_dev_ptr->sensor_id == camera_insts_[1].camera_dev_ptr->sensor_id) ||
            std::strcmp(major_frames_list_[item_current_idx].c_str(), major_frames_list_[item_current_idx2].c_str()) ==
                0) {
          std::string msg = "two cameras are the same!";
          dev::show_pfd_info("two cameras calibration", msg);
        } else if (!camera_insts_[0].camera_dev_ptr->camera_model() ||
                   !camera_insts_[1].camera_dev_ptr->camera_model()) {
          // 检测相机模型是否已经选择
          std::string msg = "please set camera model first!";
          dev::show_pfd_info("info", msg);
        } else {
          //          next_state_ = STATE_START_CALIB;
          change_next_state(std::make_shared<StateStartCalib>());
        }
      }
    }

    // 清除标定结果
    if (is_transform_valid_) {
      ImGui::SameLine();
      if (ImGui::Button("reset")) {
        is_transform_valid_ = false;
        b_need_to_update_cd_ = true;
        clear_calib_data_flag();
      }
      // tips
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("clear calib result.");
      }
    }

    if (is_transform_valid_) {
      ImGui::SameLine();
      if (ImGui::Button("save")) {
        std::vector<std::string> filters = {"calib data file (.json)", "*.json"};
        std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file", dev::data_default_path, filters));
        while (!dialog->ready()) {
          usleep(1000);
        }
        auto file_path = dialog->result();
        if (!file_path.empty()) {
          if (!save_extrinsics(file_path)) {
            printf("----- TwoCamerasCalib::draw_ui() ..... FAILED\n");
            std::string msg = "save calib data to " + file_path + " failed!";
            dev::show_pfd_info("save calib data", msg);
          } else {
            printf("----- TwoCamerasCalib::draw_ui() ..... SUCCESS\n");
            std::string msg = "save calib data to " + file_path + " ok!";
            dev::show_pfd_info("save calib data", msg);
          }
        }
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip("click to save the extrinsics.");
        }
      }
    }

    if (is_transform_valid_) {
      ImGui::SameLine();
      if (ImGui::Button("update")) {
        is_transform_valid_ = false;
        util::publish_extrinsics(ros_nh_, transform_12_, camera_insts_[0].frame_id, camera_insts_[1].frame_id,
                                 major_frames_list_[item_current_idx], major_frames_list_[item_current_idx2]);
      }
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("click to send calib data service request.");
      }
    }
  }

  // 标定数据相关操作
  //  if (next_state_ == STATE_IDLE && !calib_valid_data_vec_.empty()) {
  if (next_state_ptr_->id() == 0 && !calib_valid_data_vec_.empty()) {
    if (ImGui::Checkbox("##show_calib_data", &b_show_calib_data_)) {
      if (b_show_calib_data_) {
        // 两个设备名称不能一样
        if ((camera_insts_[0].camera_dev_ptr->sensor_id == camera_insts_[1].camera_dev_ptr->sensor_id) ||
            std::strcmp(major_frames_list_[item_current_idx].c_str(), major_frames_list_[item_current_idx2].c_str()) ==
                0) {
          std::string msg = "two cameras are the same!";
          dev::show_pfd_info("two cameras calibration", msg);
          b_show_calib_data_ = false;
        } else if (!camera_insts_[0].camera_dev_ptr->camera_model() ||
                   !camera_insts_[1].camera_dev_ptr->camera_model()) {
          // 检测相机模型是否已经选择
          std::string msg = "please set camera model first!";
          dev::show_pfd_info("info", msg);
          b_show_calib_data_ = false;
        } else {
          b_need_to_update_cd_ = true;
          // 显示AprilBoard
          april_board_ptr_->show_3d();
        }
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
        is_transform_valid_ = false;
        clear_calib_data_flag();
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
  } else if (camera_insts_[0].camera_dev_ptr && camera_insts_[1].camera_dev_ptr) {
    ImGui::SameLine();
    ImGui::TextDisabled("vaild: %zu", calib_valid_data_vec_.size());
  }

  ImGui::End();

  // 显示图像
  if (b_show_image_) {
    camera_insts_[0].img_show_dev_ptr->show_image(b_show_image_);
    camera_insts_[1].img_show_dev_ptr->show_image(b_show_image_);
  }
}

void TwoCamerasCalib::pass_major_frames(const std::vector<std::string>& major_frames) {
  // 如果传入的frame列表不为空
  if (!major_frames.empty()) {
    // 则可以把前面占位的"Empty..."都去掉
    major_frames_set_.erase("Empty... 1");
    major_frames_set_.erase("Empty... 2");
    for (auto iter = major_frames_list_.begin(); iter != major_frames_list_.end(); ++iter) {
      if (std::strcmp(iter->c_str(), "Empty... Please check /tf_static") == 0) {
        major_frames_list_.erase(iter);
        break;
      }
    }
    // 遍历传入的frame
    for (auto& f : major_frames) {
      // 尝试放入set容器
      std::pair<std::set<std::string>::iterator, bool> feedback = major_frames_set_.insert(f);
      // 如果可以被放进set，则说明是前面没有过的frame，应该被添加到vector
      if (feedback.second) {
        major_frames_list_.push_back(f);
      }
    }
  } else {
    // 如果传入的frame是空的，就放入"Empty..."占位
    std::pair<std::set<std::string>::iterator, bool> feedback =
        major_frames_set_.insert("Empty... Please check /tf_static");
    if (feedback.second) {
      major_frames_list_.emplace_back("Empty... 1");
      major_frames_list_.emplace_back("Empty... 2");
    }
  }
}

}  // namespace calibration