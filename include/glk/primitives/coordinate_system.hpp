#ifndef GLK_PRIMITIVES_COORDINATE_SYSTEM_HPP
#define GLK_PRIMITIVES_COORDINATE_SYSTEM_HPP

#include <vector>
#include <Eigen/Core>

namespace glk {

class CoordinateSystem {
public:
  CoordinateSystem() {
    vertices.emplace_back(Eigen::Vector3f::Zero());
    vertices.emplace_back(Eigen::Vector3f::UnitX());
    vertices.emplace_back(Eigen::Vector3f::Zero());
    vertices.emplace_back(Eigen::Vector3f::UnitY());
    vertices.emplace_back(Eigen::Vector3f::Zero());
    vertices.emplace_back(Eigen::Vector3f::UnitZ());

    colors.emplace_back(Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
    colors.emplace_back(Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
    colors.emplace_back(Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
    colors.emplace_back(Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
    colors.emplace_back(Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f));
    colors.emplace_back(Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f));
  }

public:
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;
};
}  // namespace glk

#endif