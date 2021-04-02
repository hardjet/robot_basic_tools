#ifndef GLK_POINTCLOUD_BUFFER_HPP
#define GLK_POINTCLOUD_BUFFER_HPP

#include <memory>
#include <string>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "glk/drawble.hpp"

namespace glk {

class GLSLShader;

class PointCloudBuffer : public Drawable{
 public:
  using Ptr = std::shared_ptr<PointCloudBuffer>;

  explicit PointCloudBuffer(const std::string& cloud_filename);
  explicit PointCloudBuffer(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
  explicit PointCloudBuffer(const std::vector<Eigen::Vector3d> &points);

  void free() override;

  void draw(glk::GLSLShader &shader) const override;

 private:
  GLuint vao{};
  GLuint vbo{};
  int stride_{};
  int num_points_{};
};

}  // namespace glk

#endif