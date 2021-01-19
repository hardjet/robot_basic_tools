#ifndef GLK_POINTCLOUD_BUFFER_HPP
#define GLK_POINTCLOUD_BUFFER_HPP

#include <memory>
#include <string>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glk {

class GLSLShader;

class PointCloudBuffer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PointCloudBuffer>;

  explicit PointCloudBuffer(const std::string& cloud_filename);
  explicit PointCloudBuffer(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
  ~PointCloudBuffer();

  void draw(glk::GLSLShader& shader) const;

 private:
  GLuint vao{};
  GLuint vbo{};
  int stride;
  int num_points;
};

}  // namespace glk

#endif