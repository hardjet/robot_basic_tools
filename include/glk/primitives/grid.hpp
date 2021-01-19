#ifndef GLK_PRIMITIVES_GRID_HPP
#define GLK_PRIMITIVES_GRID_HPP

#include <vector>
#include <Eigen/Core>

namespace glk {

class Grid {
 public:
  explicit Grid(double half_extent = 5.0, double step = 1.0) {
    for (double x = -half_extent; x <= half_extent + 1e-9;) {
      vertices.emplace_back(Eigen::Vector3f(x, -half_extent, 0.0f));
      vertices.emplace_back(Eigen::Vector3f(x, half_extent, 0.0f));
      vertices.emplace_back(Eigen::Vector3f(-half_extent, x, 0.0f));
      vertices.emplace_back(Eigen::Vector3f(half_extent, x, 0.0f));
      x += step;
    }
  }

 public:
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
};
}  // namespace glk

#endif