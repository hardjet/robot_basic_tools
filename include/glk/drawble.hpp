#ifndef GLK_DRAWABLE_HPP
#define GLK_DRAWABLE_HPP

namespace glk {

class GLSLShader;

/**
 * @brief Drawable object interface
 *
 */
class Drawable {
 public:
  virtual ~Drawable() = default;

  virtual void draw(glk::GLSLShader &shader) const = 0;
};

}  // namespace glk

#endif