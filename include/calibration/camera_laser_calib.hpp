#pragma once

#include <memory>
#include <mutex>
#include "calib_base.hpp"

namespace cv_bridge {
class CvImage;
}

namespace cv {
class Mat;
}

namespace dev {
class AprilBoard;
class Camera;
class Laser;
class ImageShow;
}  // namespace dev

namespace calibration {

class Task;

class CamLaserCalib : public BaseCalib {
 public:
  CamLaserCalib(std::shared_ptr<dev::SensorManager>& sensor_manager_ptr,
                std::shared_ptr<dev::AprilBoard>& april_board_ptr_);

  /// opengl渲染
  void draw_gl(glk::GLSLShader& shader) override;
  /// imgui绘图
  void draw_ui() override;

 private:
  /// 更新数据
  void update_data();
  /// just test
  bool find_and_draw_apriltags();

 private:
  // 资源锁
  std::mutex mtx_;
  // 标定板
  std::shared_ptr<dev::AprilBoard> april_board_ptr_;
  // 当前选中的相机对象
  std::shared_ptr<dev::Camera> cam_ptr_{nullptr};
  // 当前选中的相机对象
  std::shared_ptr<dev::Laser> laser_ptr_{nullptr};
  // 图像显示对象
  std::shared_ptr<dev::ImageShow> im_show_ptr_{nullptr};
  // 任务对象
  std::shared_ptr<Task> task_ptr_{nullptr};
  // 当前图像对象
  boost::shared_ptr<const cv_bridge::CvImage> image_ptr_{nullptr};
  // 显示使用的image
  boost::shared_ptr<const cv_bridge::CvImage> show_image_ptr_{nullptr};
  // 保存处理后显示图像
  std::shared_ptr<cv::Mat> image_mat_{nullptr};
  // 是否显示图像
  bool is_show_image_{false};
  // 是否有新数据
  bool is_new_image_{false};
};
}  // namespace calibration
