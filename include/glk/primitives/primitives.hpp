#ifndef GLK_PRIMITIVES_HPP
#define GLK_PRIMITIVES_HPP

#include <vector>
#include <memory>

namespace glk {
class Drawable;
}

namespace glk {

class Primitives {
 private:
  Primitives() { meshes.resize(NUM_PRIMITIVES, nullptr); }
  ~Primitives() { delete instance_; }

 public:
  enum PrimitiveType { ICOSAHEDRON = 0, SPHERE, CUBE, CONE, GRID, COORDINATE_SYSTEM, NUM_PRIMITIVES };

  static Primitives *instance() {
    if (instance_ == nullptr) {
      instance_ = new Primitives();
    }
    return instance_;
  }

  const glk::Drawable &primitive(PrimitiveType type);

 private:
  static Primitives *instance_;

  std::vector<std::shared_ptr<glk::Drawable>> meshes;
};
}  // namespace glk

#endif