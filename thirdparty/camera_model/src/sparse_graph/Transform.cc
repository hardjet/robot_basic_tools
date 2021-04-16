#include <camera_model/sparse_graph/Transform.h>

namespace camera_model {

Transform::Transform() {
  m_q.setIdentity();
  m_t.setZero();
}

Transform::Transform(const Eigen::Matrix4d& H) {
  m_q = Eigen::Quaterniond(H.block<3, 3>(0, 0));
  m_t = H.block<3, 1>(0, 3);
}

Eigen::Quaterniond& Transform::rotation() { return m_q; }

const Eigen::Quaterniond& Transform::rotation() const { return m_q; }

double* Transform::rotationData() { return m_q.coeffs().data(); }

const double* Transform::rotationData() const { return m_q.coeffs().data(); }

Eigen::Vector3d& Transform::translation() { return m_t; }

const Eigen::Vector3d& Transform::translation() const { return m_t; }

double* Transform::translationData() { return m_t.data(); }

const double* Transform::translationData() const { return m_t.data(); }

Eigen::Matrix4d Transform::toMatrix() const {
  Eigen::Matrix4d H;
  H.setIdentity();
  H.block<3, 3>(0, 0) = m_q.toRotationMatrix();
  H.block<3, 1>(0, 3) = m_t;

  return H;
}

}  // namespace camera_model
