#ifndef GLK_TEXT_RENDERER_HPP
#define GLK_TEXT_RENDERER_HPP

#include <iostream>
#include <map>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

#include "GL/gl3w.h"
#include "glk/glsl_shader.hpp"

namespace glk {

struct Character {
  // 字形id
  unsigned int TextureID;
  // 字形size
  glm::ivec2 Size;
  // offset from baseline
  glm::ivec2 Bearing;
  // Horizontal offset to advance to next glyph
  unsigned int Advance;
};

class TextRenderer {
 public:
  TextRenderer(const std::string& data_directory, const Eigen::Matrix<int, 2, 1>& window_size) {
    if (!shader.init(data_directory + "/shader/font")) {
      return;
    }
    shader.use();

    projection = glm::ortho(0.0f, static_cast<GLfloat>(window_size[0]), 0.0f, static_cast<GLfloat>(window_size[1]));
    glUniformMatrix4fv(glGetUniformLocation(shader.get_shader_program(), "projection"), 1, GL_FALSE,
                       glm::value_ptr(projection));

    if (FT_Init_FreeType(&ft_lib)) {
      std::cout << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
      return;
    }
    if (FT_New_Face(ft_lib, (data_directory + "/UbuntuMono-R.ttf").c_str(), 0, &ft_face)) {
      std::cout << "ERROR::FREETYPE: Failed to load font" << std::endl;
      return;
    } else {
      FT_Set_Pixel_Sizes(ft_face, 0, 48);
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

      for (unsigned char c = 0; c < 128; c++) {
        if (FT_Load_Char(ft_face, c, FT_LOAD_RENDER)) {
          std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
          continue;
        }
        GLuint texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, static_cast<int>(ft_face->glyph->bitmap.width),
                     static_cast<int>(ft_face->glyph->bitmap.rows), 0, GL_RED, GL_UNSIGNED_BYTE,
                     ft_face->glyph->bitmap.buffer);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        Character character = {texture, glm::ivec2(ft_face->glyph->bitmap.width, ft_face->glyph->bitmap.rows),
                               glm::ivec2(ft_face->glyph->bitmap_left, ft_face->glyph->bitmap_top),
                               static_cast<unsigned int>(ft_face->glyph->advance.x)};
        Characters.insert(std::pair<GLchar, Character>(c, character));
      }
      glBindTexture(GL_TEXTURE_2D, 0);
    }
    FT_Done_Face(ft_face);
    FT_Done_FreeType(ft_lib);

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }

  ~TextRenderer() {
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
  }

  void render_text(const std::string& text, float x, float y, float scale, glm::vec3 color,
                   const Eigen::Vector2i& size) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    shader.use();
    glUniform3f(glGetUniformLocation(shader.get_shader_program(), "textColor"), color.x, color.y, color.z);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(vao);

    glUniformMatrix4fv(
        glGetUniformLocation(shader.get_shader_program(), "projection"), 1, GL_FALSE,
        glm::value_ptr(glm::ortho(0.0f, static_cast<GLfloat>(size[0]), 0.0f, static_cast<GLfloat>(size[1]))));

    (scale < 0.1) ? (scale = 0.1) : scale;
    (scale > 0.5) ? (scale = 0.5) : scale;

    std::string::const_iterator text_iter;
    for (text_iter = text.begin(); text_iter != text.end(); ++text_iter) {
      Character current_ch = Characters[*text_iter];

      float xpos = x + static_cast<float>(current_ch.Bearing.x) * scale;
      float ypos = y - static_cast<float>(current_ch.Size.y - current_ch.Bearing.y) * scale;

      float w = static_cast<float>(current_ch.Size.x) * scale;
      float h = static_cast<float>(current_ch.Size.y) * scale;

      // update VBO
      float vertices[6][4] = {{xpos, ypos + h, 0.0f, 0.0f}, {xpos, ypos, 0.0f, 1.0f},
                              {xpos + w, ypos, 1.0f, 1.0f}, {xpos, ypos + h, 0.0f, 0.0f},
                              {xpos + w, ypos, 1.0f, 1.0f}, {xpos + w, ypos + h, 1.0f, 0.0f}};

      glBindTexture(GL_TEXTURE_2D, current_ch.TextureID);
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glDrawArrays(GL_TRIANGLES, 0, 6);

      x += static_cast<float>(current_ch.Advance >> 6) * scale;
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glDisable(GL_BLEND);
  }

 private:
  std::map<GLchar, Character> Characters;
  glm::mat4 projection;
  glk::GLSLShader shader;

  GLuint vao{};
  GLuint vbo{};

  FT_Library ft_lib;
  FT_Face ft_face;
};

}  // namespace glk

#endif