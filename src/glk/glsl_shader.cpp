#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <unordered_map>

#include "GL/gl3w.h"
#include "glk/glsl_shader.hpp"

namespace glk {

bool GLSLShader::init(const std::string &shader_path) {                                       // .../src/robot_basic_tools/data/shader/rainbow
  printf("----- GLSLShader::init() ..... shader_path = %s\n", shader_path.c_str());
  GLuint vertex_shader = read_shader_from_file(shader_path + ".vert", GL_VERTEX_SHADER);
  GLuint fragment_shader = read_shader_from_file(shader_path + ".frag", GL_FRAGMENT_SHADER);

  shader_program = glCreateProgram();                 // glCreateProgram创建一个空program并返回一个可以被引用的非零值（program ID）,如果创建program 对象时发生错误，则此函数返回0
  glAttachShader(shader_program, vertex_shader);      // 调用glAttachShader将着色器对象附加到program 对象
  glAttachShader(shader_program, fragment_shader);

  glLinkProgram(shader_program);                      // 连接一个program对象，附加到program的类型为GL_VERTEX_SHADER的着色器对象将用于创建在可编程顶点处理器上运行的可执行文件；类型为GL_FRAGMENT_SHADER的着色器对象用于创建将在可编程片段处理器上运行的可执行文件。

  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);

  GLint result = GL_FALSE;
  int info_log_length;

  glGetProgramiv(shader_program, GL_LINK_STATUS, &result);    // void glGetProgramiv（GLuint program,GLenum pname,GLint *params）- 从program对象返回一个参数的值
  glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &info_log_length);   // program=指定要查询的program对象; pname=指定program对象参数，包括GL_DELETE_STATUS，GL_LINK_STATUS，GL_VALIDATE_STATUS，GL_INFO_LOG_LENGTH，GL_ATTACHED_SHADERS，GL_ACTIVE_ATTRIBUTES，GL_ACTIVE_UNIFORMS，GL_ACTIVE_ATTRIBUTE_MAX_LENGTH，GL_ACTIVE_UNIFORM_MAX_LENGTH
  std::vector<char> error_message(info_log_length);
  glGetProgramInfoLog(shader_program, info_log_length, nullptr, error_message.data());

  if (result != GL_TRUE) {
    std::cerr << "error : failed to link program" << std::endl;
    std::cerr << std::string(error_message.begin(), error_message.end()) << std::endl;
    return false;
  }
  return true;
}

GLint GLSLShader::attrib(const std::string &name) {
  auto found = attrib_cache.find(name);
  if (found != attrib_cache.end()) {
    return found->second;
  }

  GLint id = glGetAttribLocation(shader_program, name.c_str());
  if (id == -1) {
    std::cerr << "warning : attrib " << name << " not found" << std::endl;
  }

  attrib_cache[name] = id;
  return id;
}

GLint GLSLShader::uniform(const std::string &name) {
  auto found = uniform_cache.find(name);
  if (found != uniform_cache.end()) {
    return found->second;
  }

  GLint id = glGetUniformLocation(shader_program, name.c_str());
  if (id == -1) {
    std::cerr << "warning : uniform " << name << " not found" << std::endl;
  }

  uniform_cache[name] = id;
  return id;
}

GLuint GLSLShader::read_shader_from_file(const std::string &filename, GLuint shader_type) {
  GLuint shader_id = glCreateShader(shader_type);
  std::ifstream ifs(filename);
  if (!ifs) {
    std::cerr << "error: failed to open " << filename << std::endl;
    return GL_FALSE;
  }

  std::stringstream sst;
  sst << ifs.rdbuf();
  std::string source = sst.str();

  GLint result = GL_FALSE;
  int info_log_length = 0;

  char const *source_ptr = source.c_str();
  glShaderSource(shader_id, 1, &source_ptr, nullptr);
  glCompileShader(shader_id);

  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &info_log_length);

  std::vector<char> error_message(info_log_length);
  glGetShaderInfoLog(shader_id, info_log_length, nullptr, error_message.data());

  if (result != GL_TRUE) {
    std::cerr << "error : failed to compile shader " << filename << std::endl;
    std::cerr << std::string(error_message.begin(), error_message.end()) << std::endl;
  }

  return shader_id;
}

}  // namespace glk
