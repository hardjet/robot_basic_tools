#include <fstream>
#include <cv_bridge/cv_bridge.h>

#include "imgui.h"
#include "portable-file-dialogs.h"
#include "nlohmann/json.hpp"

#include "glk/glsl_shader.hpp"
#include "glk/primitives/primitives.hpp"

#include "dev/april_board.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "dev/camera.hpp"
#include "dev/util.hpp"

#include "algorithm/util.h"
#include "calibration/task_back_ground.hpp"
#include "calibration/camera_calib.hpp"

#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"
#include "camera_model/camera_models/Camera.h"

#include "camera_model/calib/CameraCalibration.h"

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
CameraCalib::CameraCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                         std::shared_ptr<dev::AprilBoard>& april_board_ptr)
    : BaseCalib(sensor_manager_ptr), april_board_ptr_(april_board_ptr) {
  // 图像显示
  image_imshow_ptr_ = std::make_shared<dev::ImageShow>();
  // 后台任务
  task_ptr_ = std::make_shared<calibration::Task>();
}
void CameraCalib::draw_ui() {
  if (!b_show_window_) {
    return;
  }
  // 新建窗口
  ImGui::Begin("one camera Calibration", &b_show_window_, ImGuiWindowFlags_AlwaysAutoResize);
  // 相机选择
  draw_sensor_selector<dev::Camera>("camera", dev::CAMERA, cam_dev_ptr_);
  //ImGui::SameLine();
//  if (ImGui::Button("camera calibration"))
//  {
//    // 选择加载文件路径
//    std::string msg = "please set camera model first!";
//    dev::show_pfd_info("info", msg);
//  }

  if (cam_dev_ptr_) {
    // 选择是否显示图像
    if (ImGui::Checkbox("show image", &b_show_image_)) {
      if (b_show_image_) {
        image_imshow_ptr_->enable("calib: camera", false);
      } else {
        image_imshow_ptr_->disable();
      }
    }
  }
  // 闲置状态下才可以设置
  if (next_state_ == STATE_IDLE) {
    draw_calib_params();
  }

// 设置变换矩阵参数
//  draw_ui_transform();

  // 标定逻辑
  calibration();

  if (next_state_ == STATE_IDLE) {
    if (ImGui::Button("start"))
    {
      // 检测相机模型是否已经选择
      if (!cam_dev_ptr_->camera_model()) {
        std::string msg = "please set camera model first!";
        dev::show_pfd_info("info", msg);
      } else {
        b_show_calib_data_ = false;
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
  // 标定数据相关操作
  if (next_state_ == STATE_IDLE && !calib_valid_data_vec_.empty()) {
    if (ImGui::Checkbox("##show_calib_data", &b_show_calib_data_)) {
      if (b_show_calib_data_) {
        b_need_to_update_cd_ = true;
        // 显示AprilBoard
        april_board_ptr_->show_3d();
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
  } else if (cam_dev_ptr_) {
    ImGui::SameLine();
    ImGui::TextDisabled("vaild: %zu", calib_valid_data_vec_.size());
  }
  ImGui::End();
  if (b_show_image_) {
    image_imshow_ptr_->show_image(b_show_image_);
  }
}

void CameraCalib::draw_gl(glk::GLSLShader& shader) {
  if (!b_show_window_) {
    return;
  }
}
/// 设置标定参数
void CameraCalib::draw_calib_params() {
  const double min_v = 0.;
  ImGui::Separator();
  ImGui::Text("calibration params:");

  // 设定宽度
  ImGui::PushItemWidth(80);
  ImGui::DragScalar("chess_size(mm)", ImGuiDataType_Double, &chess_size_, 0.1, nullptr, nullptr, "%.2f");
  // tips
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("set chess board grid size.");
  }
}
bool CameraCalib::load_calib_data(const std::string& file_path) {}
//更新3D图像点信息
void CameraCalib::update_3d_show() {
  // 更新显示图象
  image_imshow_ptr_->update_image(show_cam_cv_img_ptr_);
}
void CameraCalib::update_data() {
  std::lock_guard<std::mutex> lock(mtx_);
  // 图像
  if (!cv_image_ptr_) {
    cv_image_ptr_ = cam_dev_ptr_->data();
    if (cv_image_ptr_) {
      is_new_image_ = true;
    }
  } else {
    auto image = cam_dev_ptr_->data();
    if (cv_image_ptr_->header.stamp.nsec != image->header.stamp.nsec) {
      cv_image_ptr_ = image;
      is_new_image_ = true;
    }
  }
}
bool CameraCalib::get_pose_and_points() {
  // 检测到的角点空间坐标
  std::vector<cv::Point2f> imagePoints;
  // 检测到的角点图像坐标
  std::vector<cv::Point3f> objectPoints;
  // 相机坐标系到 april board 坐标系的变换
  Eigen::Matrix4d T_ac;
  // 当前处理的图像
  boost::shared_ptr<const cv_bridge::CvImage> cur_image{nullptr};
  // 获取当前图像对象
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cur_image.reset(new const cv_bridge::CvImage(cv_image_ptr_->header, cv_image_ptr_->encoding, cv_image_ptr_->image));
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
  cv::imshow("test",img_show);
  cv::waitKey(10);
  //通过图像得到april board中角点的位置和图像点
  if (april_board_ptr_->board->computeObservation(img, img_show, objectPoints, imagePoints)) {
    //每一张图像进入后都要进行外参计算
    cam_dev_ptr_->camera_model()->estimateExtrinsics(objectPoints, imagePoints, T_ac, img_show);
    calib_data_.timestamp = cur_image->header.stamp.toSec();
    calib_data_.t_ac = T_ac.block(0, 3, 3, 1);
    Eigen::Matrix3d R_ac = T_ac.block(0, 0, 3, 3);
    Eigen::Quaterniond q_ac(R_ac);
    calib_data_.q_ac = q_ac;
    //这种写法对不对？
    calib_data_.imagePoints = imagePoints;
    calib_data_.objectPoints = objectPoints;
    //用来显示图像
    show_cam_cv_img_ptr_.reset(new const cv_bridge::CvImage(cur_image->header, cur_image->encoding, img_show));
    return true;
  } else {
    return false;
  }
}
//检查缓存的点云数据并进行保存
void CameraCalib::check_and_save() {
  bool b_need_to_save = true;
  // 逐个检测角度值
  for (auto& data : calib_valid_data_vec_) {
    double theta = 2 * std::acos((data.q_ac.inverse() * calib_data_vec_.at(2).q_ac).w());
    // 保证间隔x以上
    if (theta < DEG2RAD_RBT(between_angle_)) {
      b_need_to_save = false;
      break;
    }
  }
  if (b_need_to_save) {
    // 取第3帧保存
    calib_valid_data_vec_.push_back(calib_data_vec_.at(2));
    printf("!!!!!!!!!!!!!tz: %f saved!\n", calib_valid_data_vec_.back().t_ac.z());
  }
}
//核心相机矫正启动函数
void CameraCalib::calibration() {
  std::cout<<"calibration()"<<std::endl;
  update_data();
  switch (cur_state_) {
    case STATE_IDLE:
      cur_state_ = next_state_;
      break;
    case STATE_START:
      if (is_new_image_) {
        is_new_image_ = false;
        cur_state_ = STATE_GET_POSE_AND_PTS;
      } else {
        cur_state_ = STATE_IDLE;
      }
      break;
    case STATE_GET_POSE_AND_PTS:
      // 执行任务
      if (task_ptr_->do_task("get_pose_and_points", std::bind(&CameraCalib::get_pose_and_points, this))) {  // NOLINT
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          update_3d_show();
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
        double dist = (calib_data_vec_.at(0).t_ac - calib_data_.t_ac).norm();
        // 四元数的转角是原本的1/2
        double theta = 2 * std::acos((calib_data_vec_.at(0).q_ac.inverse() * calib_data_.q_ac).w());
        std::cout << "dist:" << dist << ", theta:" << RAD2DEG_RBT(theta) << std::endl;
        // 抖动小于1cm与0.8°
        if (dist < 0.01 && theta < DEG2RAD_RBT(0.8)) {
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
      if (task_ptr_->do_task("calc", std::bind(&CameraCalib::calc, this))) {  // NOLINT
        // 结束后需要读取结果
        if (task_ptr_->result<bool>()) {
          //update_related_pose();
         // update_ui_transform();
          cur_state_ = STATE_IDLE;
        }
      }
      break;
  }
}
//核心矫正算法函数
bool CameraCalib::calc()
{
  std::cout<<"变成一道光"<<std::endl;
//  cv::Size m_boardSize;
//  camera_model::CameraCalibration cam_cal
//      (cam_dev_ptr_->sensor_type);
//  cam_cal_.clear();
//  //准备标定数据
//  std::vector<std::vector<cv::Point2f> > m_imageGoodPoints;
//  std::vector<std::vector<cv::Point3f> > m_sceneGoodPoints;
//  for (const auto& data : calib_data_vec_)
//  {
//    cam_cal_.addChessboardData(data.imagePoints,data.objectPoints);
//  }
//  cam_cal_.calibrate();
  return true;
}
}  // namespace calibration