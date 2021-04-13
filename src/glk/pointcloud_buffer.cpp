#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "glk/glsl_shader.hpp"
#include "glk/pointcloud_buffer.hpp"

namespace glk {

PointCloudBuffer::PointCloudBuffer(const std::string &cloud_filename) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile(cloud_filename, *cloud)) {
    std::cerr << "error: failed to load " << cloud_filename << std::endl;
    num_points_ = 0;
    return;
  }

  stride_ = sizeof(pcl::PointXYZI);
  num_points_ = int(cloud->size());

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(pcl::PointXYZI), cloud->points.data(), GL_STATIC_DRAW);
}

PointCloudBuffer::PointCloudBuffer(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) {
  stride_ = sizeof(pcl::PointXYZI);
  num_points_ = cloud->size();

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(pcl::PointXYZI), cloud->points.data(), GL_STATIC_DRAW);
}

PointCloudBuffer::PointCloudBuffer(const std::vector<Eigen::Vector3d> &points) {
  num_points_ = int(points.size());

  stride_ = sizeof(Eigen::Vector3f);
  // 将Vector3d转为Vector3f
  std::vector<Eigen::Vector3f> pts(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    pts.at(i) = points.at(i).cast<float>();
  }

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, pts.size() * stride_, pts.data(), GL_STATIC_DRAW);
}

void PointCloudBuffer::free() {
  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}

void PointCloudBuffer::draw(glk::GLSLShader &shader) const {
  if (num_points_ == 0) {
    return;
  }

  GLint position_loc = shader.attrib("vert_position");

  glBindVertexArray(vao);
  glEnableVertexAttribArray(position_loc);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride_, nullptr);

  glDrawArrays(GL_POINTS, 0, num_points_);

  glDisableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

}  // namespace glk
