#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace algorithm {

struct Oberserve {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Oberserve() {
    tag_pose_q_ca = Eigen::Quaterniond(1, 0, 0, 0);
    tag_pose_t_ca = Eigen::Vector3d::Zero();
  }
  // april board在相机坐标系下的位姿
  Eigen::Quaterniond tag_pose_q_ca;
  Eigen::Vector3d tag_pose_t_ca;
  // 该时刻激光点云的数据
  std::vector<Eigen::Vector3d> points;
  // 该时刻激光点云的数据
  std::vector<Eigen::Vector3d> points_on_line;
};

void LineFittingCeres(const std::vector<Eigen::Vector3d>& Points, Eigen::Vector2d& Line);
void CamLaserCalClosedSolution(const std::vector<Oberserve>& obs, Eigen::Matrix4d& Tlc);
void CamLaserCalibration(const std::vector<Oberserve>& obs, Eigen::Matrix4d& Trc, bool use_linefitting_data = true,
                         bool use_boundary_constraint = false);

}  // namespace algorithm
