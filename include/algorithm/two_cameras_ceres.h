#pragma once

#include <vector>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include "opencv2/core/types.hpp"

namespace camera_model {
class Camera;
}

namespace algorithm {
namespace TwoCamerasCalib {

struct Observation {
  // 图像上的点
  std::vector<Eigen::Vector2d> image_points;
  // 空间点
  std::vector<Eigen::Vector3d> object_points;
};

/**
 * 使用ceres计算位姿
 * @param camera_ptr
 * @param obs
 * @param T_21 相机1到相机2的变换矩阵
 */
bool calibrate(const boost::shared_ptr<camera_model::Camera> &camera_ptr, const Observation &obs,
               Eigen::Matrix4d &T_21);

}  // namespace TwoCamerasCalib
}  // namespace algorithm