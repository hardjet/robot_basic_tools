#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
// #include <cstdint>

namespace camera_model {

class Transform {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Transform();
  explicit Transform(const Eigen::Matrix4d& H);

  Eigen::Quaterniond& rotation();
  const Eigen::Quaterniond& rotation() const;
  double* rotationData();
  const double* rotationData() const;

  Eigen::Vector3d& translation();
  const Eigen::Vector3d& translation() const;
  double* translationData();
  const double* translationData() const;

  Eigen::Matrix4d toMatrix() const;

 private:
  Eigen::Quaterniond m_q;
  Eigen::Vector3d m_t;
};

}  // namespace camera_model

#endif
