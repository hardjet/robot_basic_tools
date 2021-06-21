#include <fstream>
#include <cv_bridge/cv_bridge.h>

#include "imgui.h"
#include "portable-file-dialogs.h"
#include "nlohmann/json.hpp"

#include "glk/glsl_shader.hpp"
#include "glk/primitives/primitives.hpp"

#include "dev/april_board.hpp"
#include "dev/chess_board.hpp"
#include "dev/blob_board.hpp"
#include "dev/sensor_manager.hpp"
#include "dev/image_show.hpp"
#include "dev/camera.hpp"
#include "dev/util.hpp"

#include "algorithm/util.h"
#include "calibration/task_back_ground.hpp"
#include "calibration/camera_calib.hpp"

#include "camera_model/camera_models/Camera.h"
#include "camera_model/camera_models/CameraFactory.h"
#include "camera_model/apriltag_frontend/GridCalibrationTargetAprilgrid.hpp"
#include "camera_model/chessboard/Chessboard.h"

#include <opencv2/calib3d.hpp>
// ---- 相机标定状态
// 空闲
#define STATE_IDLE 0
// 启动
#define STATE_START 1
// 获取相机位姿
#define STATE_GET_POSE_AND_PTS 2
// 检查数据稳定性,连续5帧姿态没有大的变化，取中间帧作为候选
#define STATE_CHECK_STEADY 3
// 开始标定
#define STATE_START_CALIB 4
// 在标定过程中
#define STATE_IN_CALIB 5

namespace calibration {
CameraCalib::CameraCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                         std::shared_ptr<dev::AprilBoard>& april_board_ptr,
                         std::shared_ptr<dev::chessboard>& chess_board_ptr)
    : BaseCalib(sensor_manager_ptr), april_board_ptr_(april_board_ptr), chess_board_ptr_(chess_board_ptr) {
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
  ImGui::Begin("Monocular camera Calibration", &b_show_window_, ImGuiWindowFlags_AlwaysAutoResize);
  // 相机选择
  draw_sensor_selector<dev::Camera>("camera", dev::CAMERA, cam_dev_ptr_);
  if (cam_dev_ptr_) {
    // 从文件加载标定数据
    ImGui::SameLine();
    if (ImGui::Button("R")) {
      if (!cam_dev_ptr_->camera_model()) {
        std::string msg = "please set camera model first!";
        dev::show_pfd_info("info", msg);
      } else {
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
    }
    // tips
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("load from .json");
    }
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
  if (cam_dev_ptr_) {
    //将信息保存到文件中
    if (!calib_valid_data_vec_.empty()) {
      ImGui::SameLine();
      // 保存标定数据
      if (ImGui::Button("SAVE PATH")) {
        // 选择保存文件路径
        std::vector<std::string> filters = {"create calib result (.yaml)", "*.yaml"};
        std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file", dev::data_default_path, filters));
        while (!dialog->ready()) {
          usleep(1000);
        }
        save_yaml_path = dialog->result();
      }
      // tips
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("save calib result to .yaml file");
      }
    }
  }
  if (cam_dev_ptr_) {
    //  选择是否显示图像
    if (ImGui::Checkbox("Display calibration image", &b_show_image_)) {
      if (b_show_image_) {
        image_imshow_ptr_->enable("Display calibration image", false);
      } else {
        image_imshow_ptr_->disable();
      }
    }
    if (cam_dev_ptr_->camera_model()) {
      // 闲置状态下才可以设置
      if (next_state_ == STATE_IDLE) {
        draw_calib_params();
      }
      calibration();
    }
    if (next_state_ == STATE_IDLE) {
      if (ImGui::Button("START")) {
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
      if (ImGui::Button("STOP")) {
        next_state_ = STATE_IDLE;
      }
    }
  }
  // 标定状态只需要设定一次
  if (cur_state_ == STATE_IN_CALIB) {
    next_state_ = STATE_IDLE;
  }
  // 大于7帧数据就可以开始进行标定操作了
  if (calib_valid_data_vec_.size() > 7) {
    ImGui::SameLine();
    // 开始执行标定
    if (ImGui::Button("CALIB & SAVE ")) {
      next_state_ = STATE_START_CALIB;
    }
  }
  // 标定数据相关操作
  if (next_state_ == STATE_IDLE && !calib_valid_data_vec_.empty()) {
    if (ImGui::Checkbox("##show_calib_data", &b_show_calib_data_)) {
      if (b_show_calib_data_) {
        b_need_to_update_cd_ = true;
        // 显示棋盘格
        if (USE_APRIL_BOARD) {
          april_board_ptr_->show_3d();
        } else {
          chess_board_ptr_->show_3d();
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
    ImGui::TextDisabled("Number of valid pictures: %d/%zu", selected_calib_data_id_, calib_valid_data_vec_.size());
  } else if (cam_dev_ptr_) {
    ImGui::SameLine();
    ImGui::TextDisabled("Number of valid pictures: %zu", calib_valid_data_vec_.size());
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
  draw_calib_data(shader);
}
/// 设置标定参数
void CameraCalib::draw_calib_params() {
  //  ImGui::Separator();
  //  const char* camera_type[] = {"KANNALA_BRANDT", "MEI", "PINHOLE"};
  //  ImGui::Text("camera type:%s", camera_type[cam_dev_ptr_->camera_model()->modelType()]);
  draw_ui_params();
}
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
  //是否灰度翻转
  bool USE_IMAGE_INVERSE = false;

  if(USE_IMAGE_INVERSE)
  {
    for (int row=0;row<img.rows;row++)
    {
      for (int col=0;col<img.cols;col++)
      {
        int px_value = img.at<uchar>(row,col);
        img.at<uchar>(row, col) = 255 - px_value;
      }
    }
  }
  //通过图像得到april board中角点的位置和图像点
  objectPoints.clear();
  imagePoints.clear();
  bool result = false;
  //判断是不是用Aprilboard标定板
  if (USE_APRIL_BOARD) {
    // 利用A标定板查找对应点
    if (april_board_ptr_->board->computeObservation(img, img_show, objectPoints, imagePoints)) {
      result = true;  // 找到
    } else {
      result = false;  // 未找到
    }
  } else if(USE_BOARD_BOARD)
  {
    cv::Size boardSize  =blob_board_ptr_->get_board_size();
    // 默认的就是对称圆环标定
    if(cv::findCirclesGrid(img,boardSize,imagePoints))
    {
      result = true;
      double m_tag_size =blob_board_ptr_->get_tag_size_();
      objectPoints.clear();
      for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
          objectPoints.push_back(cv::Point3f(i * m_tag_size, j * m_tag_size, 0.0));
        }
      }
    }
    else
    {
      result = false;
    }
  }
  else
  //是否使用标准的棋盘格检测角点
  {
    if (chess_board_ptr_->board->findCorners(img, objectPoints, imagePoints, true)) {
      // 判断是不是找到特征点
      result = true;  // 找到
    } else {
      result = false;  // 未找到
    }
  }
  if (result) {
    //每一张图像进入后都要进行外参计算
    auto pi_cam = camera_model::CameraFactory::instance()->generateCamera(
        camera_model::Camera::ModelType::PINHOLE, "pi_camera", cam_dev_ptr_->camera_model()->imageSize());
    std::vector<std::vector<cv::Point3f>> all_objectPoints;
    std::vector<std::vector<cv::Point2f>> all_imagePoints;
    all_objectPoints.push_back(objectPoints);
    all_imagePoints.push_back(imagePoints);
    pi_cam->setInitIntrinsics(all_objectPoints, all_imagePoints);
    pi_cam->estimateExtrinsics(objectPoints, imagePoints, T_ac, img_show);
    calib_data_.timestamp = cur_image->header.stamp.toSec();
    calib_data_.t_ac = T_ac.block(0, 3, 3, 1);
    Eigen::Matrix3d R_ac = T_ac.block(0, 0, 3, 3);
    Eigen::Quaterniond q_ac(R_ac);
    calib_data_.q_ac = q_ac;
    calib_data_.imagePoints = imagePoints;
    calib_data_.objectPoints = objectPoints;
    calib_data_.pic_show =img;
    //用来显示图像
    show_cam_cv_img_ptr_.reset(new const cv_bridge::CvImage(cur_image->header, cur_image->encoding, img_show));
    return true;
  } else {
    return false;  //直接退出
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
        // std::cout << "dist:" << dist << ", theta:" << RAD2DEG_RBT(theta) << std::endl;
        //  抖动小于1cm与0.8°
        if (dist < 0.5 && theta < DEG2RAD_RBT(0.8)) {
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
          // update_related_pose();
          //  update_ui_transform();
          cur_state_ = STATE_IDLE;
        }
      }
      break;
  }
}
//核心矫正算法函数
bool CameraCalib::calc() {
  cv::Size boardSize{0, 0};
  float tag_size{0.0};
  if (USE_APRIL_BOARD) {
    //获取标定板的大小
    boardSize.width = (int)april_board_ptr_->board->cols();
    boardSize.height = (int)april_board_ptr_->board->rows();
    tag_size = (float)april_board_ptr_->board->get_tagsize() * 100;
  } else {
    boardSize.width = chess_board_ptr_->board->rows();
    boardSize.height = chess_board_ptr_->board->cols();
    tag_size = (float)chess_board_ptr_->board->get_tagsize() * 100;
  }
  //类初始化
  camera_model::CameraCalibration cam_cal_(cam_dev_ptr_->camera_model()->modelType(),
                                           cam_dev_ptr_->camera_model()->cameraName(),
                                           cam_dev_ptr_->camera_model()->imageSize(), boardSize, tag_size);
  //显示标定板加载信息
  std::cout << "boardSize.width:" << boardSize.width << std::endl;
  std::cout << "boardSize.height:" << boardSize.height << std::endl;
  std::cout << "tag_size:" << tag_size << "mm" << std::endl;
  //对相机的缓存点进行初始化
  cam_cal_.clear();
  //从所有数据中加载特征信息
  for (const auto& data : calib_valid_data_vec_) {
    cam_cal_.addChessboardData(data.imagePoints, data.objectPoints);
  }
  //进行矫正
  cam_cal_.calibrate();
  //相机参数保存
  cam_cal_.camera()->writeParametersToYamlFile(save_yaml_path);
  //相机准备参数
  std::vector<double> inst_param_temp;
  //从矫正好的相机中读取参数
  cam_cal_.camera()->writeParameters(inst_param_temp);
  //在相机设备中更新参数
  cam_dev_ptr_->camera_model()->readParameters(inst_param_temp);
  //更新显示参数
  inst_params_ =inst_param_temp;
  //更新相机参数
  cam_dev_ptr_->update_params();
  // 更新相机坐标系到 april board 坐标系的变换
  Eigen::Matrix4d T_ac;
  for (auto& data : calib_valid_data_vec_) {
    cam_dev_ptr_->camera_model()->estimateExtrinsics(data.objectPoints, data.imagePoints, T_ac, data.pic_show);
    data.t_ac = T_ac.block(0, 3, 3, 1);
    Eigen::Matrix3d R_ac = T_ac.block(0, 0, 3, 3);
    Eigen::Quaterniond q_ac(R_ac);
    data.q_ac = q_ac;
  }
  std::cout<<"End of camera calibration！"<<std::endl;
  return true;
}
void CameraCalib::draw_ui_params() {
  static double const_0 = 0.0;
  // ImGui::BeginGroup();
  //读取相机参数保存到容器中
  cam_dev_ptr_->camera_model()->writeParameters(inst_params_);
  ImGui::Text("width:%d", cam_dev_ptr_->camera_model()->imageWidth());
  ImGui::SameLine();
  ImGui::Text("height:%d", cam_dev_ptr_->camera_model()->imageHeight());
  ImGui::Separator();
  ImGui::Text("params:");
  // 设定宽度
  ImGui::PushItemWidth(80);
  switch (cam_dev_ptr_->camera_model()->modelType()) {
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
    case camera_model::Camera::ModelType::PINHOLE_FULL:
      // 新行
      ImGui::DragScalar("k1", ImGuiDataType_Double, &inst_params_[0], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("k2", ImGuiDataType_Double, &inst_params_[1], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("k3", ImGuiDataType_Double, &inst_params_[2], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("k4", ImGuiDataType_Double, &inst_params_[3], 0.001, nullptr, nullptr, "%.6f");
      // 新行
      ImGui::DragScalar("k5", ImGuiDataType_Double, &inst_params_[4], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("k6", ImGuiDataType_Double, &inst_params_[5], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("p1", ImGuiDataType_Double, &inst_params_[6], 1.0, &const_0, nullptr, "%.2f");
      ImGui::SameLine();
      ImGui::DragScalar("p2", ImGuiDataType_Double, &inst_params_[7], 1.0, &const_0, nullptr, "%.2f");
      // 新行
      ImGui::DragScalar("fx", ImGuiDataType_Double, &inst_params_[8], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("fy", ImGuiDataType_Double, &inst_params_[9], 0.001, nullptr, nullptr, "%.6f");
      ImGui::SameLine();
      ImGui::DragScalar("cx", ImGuiDataType_Double, &inst_params_[10], 1.0, &const_0, nullptr, "%.2f");
      ImGui::SameLine();
      ImGui::DragScalar("cy", ImGuiDataType_Double, &inst_params_[11], 1.0, &const_0, nullptr, "%.2f");
      break;
    default:
      break;
  }
  ImGui::PopItemWidth();
  // ImGui::EndGroup();
}
void CameraCalib::draw_calib_data(glk::GLSLShader& shader) {
  if (!b_show_calib_data_) {
    return;
  }
  // 更新显示直线
  if (b_need_to_update_cd_) {
    b_need_to_update_cd_ = false;
  }
  auto& cur_calib_data = calib_valid_data_vec_[selected_calib_data_id_ - 1];
  // 设置april_board在相机1坐标下的位姿
  //(注意理解：在PnP方法中，世界坐标系中的点可以理解成第一个相机坐标系下的点，所以这里用的是c1a。
  // 如果已知另一位姿态下相机拍摄到图像中的点(看到相同的世界点)
  // 便可以pnp方法求出  图像点 = 转换矩阵 * 世界点  |转换矩阵 便是 从第一帧图像坐标系转换到当前图像坐标系 T_c1a_cur
  // )
  //在这里理解可以当前april_tag的位姿 （在第一帧图像坐标系下）
  //但是OpenCV中SovePnP算法中输出的rvec - 输出的旋转向量。使坐标点从世界坐标系旋转到相机坐标系 （R_cur_c1）
  //                            tvec - 输出的平移向量。使坐标点从世界坐标系平移到相机坐标系
  Eigen::Matrix4f T_c1a = Eigen::Matrix4f::Identity();
  T_c1a.block<3, 3>(0, 0) = cur_calib_data.q_ac.toRotationMatrix().transpose().cast<float>();
  T_c1a.block<3, 1>(0, 3) = -T_c1a.block<3, 3>(0, 0) * cur_calib_data.t_ac.cast<float>();

  // 计算aprilboard在世界坐标系下的位姿
  Eigen::Matrix4f T_wc1 = cam_dev_ptr_->get_sensor_pose();
  Eigen::Matrix4f T_wa = T_wc1 * T_c1a;
  // 设置位姿
  if (USE_APRIL_BOARD) {
    april_board_ptr_->set_pose(T_wa);
  } else {
    chess_board_ptr_->set_pose(T_wa);
  }
  if (cur_calib_data.b_need_calc) {
    cur_calib_data.b_need_calc = false;
    // 1. 选取标定板上三个不在一条线的点(标定板坐标系下)
    std::vector<Eigen::Vector3d> pts_on_april_board(3);
    pts_on_april_board[0] = Eigen::Vector3d{0., 0., 0.};
    pts_on_april_board[1] = Eigen::Vector3d{0.1, 0.2, 0.};
    pts_on_april_board[2] = Eigen::Vector3d{0.2, 0.1, 0.};
    Eigen::Matrix3d R_ca;
    Eigen::Vector3d t_ca;
    // 2. 计算变换矩阵
    R_ca = cur_calib_data.q_ac.toRotationMatrix().transpose();
    t_ca = -R_ca * cur_calib_data.t_ac;
    // 将标定板上三个不在一条线的点变换到相机坐标系下
    std::vector<Eigen::Vector3d> pts_on_april_board_in_cam;
    for (const auto& p : pts_on_april_board) {
      pts_on_april_board_in_cam.emplace_back(R_ca * p + t_ca);
    }
    // 3. 计算平面方程 ax + by + cz + d = 0
    Eigen::Vector4d plane_params = algorithm::plane_from_3pts(
        pts_on_april_board_in_cam[0], pts_on_april_board_in_cam[1], pts_on_april_board_in_cam[2]);

    // 4. 计算图像上落在标定板上的点
    for (const auto& p : cur_calib_data.imagePoints)
    {
      Eigen::Vector3d p_in_space;
      cam_dev_ptr_->camera_model()->liftProjective(Eigen::Vector2d{p.x, p.y}, p_in_space);
      Eigen::Vector3d p_on_board;
      Eigen::Vector3d p_in_space_temp;
      // 计算交点
      bool res = algorithm::plane_line_intersect_point(plane_params.head(3), pts_on_april_board_in_cam[1],
                                                       p_in_space, p_in_space, p_on_board);
      if (res)
      {
        Eigen::Vector3d pts_on_board_temp;
        pts_on_board_temp[0] =p_on_board[1];
        pts_on_board_temp[1] =p_on_board[0];
        pts_on_board_temp[2] =p_on_board[2];
        cur_calib_data.pts_on_board.emplace_back(p_on_board);
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
      for (const auto& pt : cur_calib_data.pts_on_board) {
        draw_points(cam_dev_ptr_->get_sensor_pose(), pt);
      }
    }
  }
}
/*
 * json点与point2f点之间的转换关系
 */
//将Point2f转换成json
static nlohmann::json convert_pts2f_to_json(const std::vector<cv::Point2f>& pts) {
  std::vector<nlohmann::json> json_pts(pts.size());
  for (unsigned int idx = 0; idx < pts.size(); ++idx) {
    json_pts.at(idx) = {pts[idx].x, pts[idx].y};
  }
  return json_pts;
}
//将Point3f转换成json
static nlohmann::json convert_pts3f_to_json(const std::vector<cv::Point3f>& pts) {
  std::vector<nlohmann::json> json_pts(pts.size());
  for (unsigned int idx = 0; idx < pts.size(); ++idx) {
    json_pts.at(idx) = {pts[idx].x, pts[idx].y, pts[idx].z};
  }
  return json_pts;
}
//将json文件保存为Point2f数据
static void convert_json_to_pts2f(std::vector<cv::Point2f>& pts, nlohmann::json& js) {
    std::vector<std::vector<float>> v;
    js.get_to<std::vector<std::vector<float>>>(v);
    pts.resize(v.size());
    for (uint j = 0; j < pts.size(); j++)
    {
      pts.at(j) = cv::Point2f{v[j][0], v[j][1]};
    }
}
//将json文件保存为Point3f数据
static void convert_json_to_pts3f(std::vector<cv::Point3f>& pts, nlohmann::json& js) {
  std::vector<std::vector<float>> v;
  js.get_to<std::vector<std::vector<float>>>(v);
  pts.resize(v.size());
  for (uint j = 0; j < pts.size(); j++) {
    pts.at(j) = cv::Point3f{v[j][0], v[j][1], v[j][2]};
  }
}
bool CameraCalib::save_calib_data(const std::string& file_path) {
  if (calib_valid_data_vec_.empty()) {
    return false;
  }
  nlohmann::json js_whole;
  js_whole["type"] = "Monocular camera Calibration";
  // 整理数据
  std::vector<nlohmann::json> js_data;
  for (const auto& data : calib_valid_data_vec_) {
    nlohmann::json js = {{"timestamp", data.timestamp},
                         {"q_ac", {data.q_ac.x(), data.q_ac.y(), data.q_ac.z(), data.q_ac.w()}},
                         {"t_ac", {data.t_ac.x(), data.t_ac.y(), data.t_ac.z()}},
                         {"imagePoints", convert_pts2f_to_json(data.imagePoints)},
                         {"objectPoints", convert_pts3f_to_json(data.objectPoints)}};
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
bool CameraCalib::load_calib_data(const std::string& file_path) {
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
  if (!js_whole.contains("type") || js_whole["type"] != "Monocular camera Calibration") {
    std::cout << "wrong file type!" << std::endl;
    return false;
  }
  // 清空之前的数据
  calib_valid_data_vec_.clear();
  // 加载数据
  for (const auto& data : js_whole["data"].items()) {
    CalibData d;
    std::vector<double> v;
    d.timestamp = data.value().at("timestamp").get<double>();
    data.value().at("q_ac").get_to<std::vector<double>>(v);
    d.q_ac.x() = v[0];
    d.q_ac.y() = v[1];
    d.q_ac.z() = v[2];
    d.q_ac.w() = v[3];
    v.clear();
    data.value().at("t_ac").get_to<std::vector<double>>(v);
    d.t_ac = Eigen::Map<Eigen::VectorXd>(v.data(), 3);
    v.clear();
    convert_json_to_pts2f(d.imagePoints, data.value().at("imagePoints"));
    // 加载直线上的点
    convert_json_to_pts3f(d.objectPoints, data.value().at("objectPoints"));
    calib_valid_data_vec_.push_back(d);
  }
  return true;
}
}  // namespace calibration
