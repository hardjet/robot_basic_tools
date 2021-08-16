#include <iostream>
#include <unistd.h>
#include <unordered_map>

#include "imgui.h"
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

// 必须要在 #include <GLFW/glfw3.h> 之前
#include "GL/gl3w.h"
#include <GLFW/glfw3.h>
#include "guik/imgui_application.hpp"

namespace guik {

Application::Application() : window(nullptr) {}

Application::~Application() {
  if (!window) {
    return;
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}

// dirty implementation
std::unordered_map<GLFWwindow *, Application *> appmap;

void fb_size_callback(GLFWwindow *window, int width,
                      int height) {  // 窗口变化的回调函数，参数=glfw的窗口，一个宽度和高度
  appmap[window]->framebuffer_size_callback(Eigen::Vector2i(width, height));
}

bool Application::init(const char *window_name, const char *imgui_config_path, const Eigen::Vector2i &size,
                       const char *glsl_version) {
  glfwSetErrorCallback([](int err, const char *desc) {
    std::cerr << "glfw error " << err << ": " << desc << std::endl;
  });                                                       // 匿名函数 + 函数类型
  if (!glfwInit()) {                                        // 初始化GLFW
    std::cerr << "failed to initialize GLFW" << std::endl;  // 初始化失败执行的操作
    return false;
  }

  glfwWindowHint(
      GLFW_CONTEXT_VERSION_MAJOR,
      3);  // 为下一次glfwCreateWindow函数的调用设置hints，对上下文环境进行相应的配置，调用通常在glfwCreateWindow前，而且这些hints设置以后将会保持不变
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);  // 设置OpenGL的最低主版本，设置OpenGL的最低副版本

  window = glfwCreateWindow(
      size[0], size[1], window_name, nullptr,
      nullptr);  // GLFWwindow* widnow;
                 // 在构造函数中被初始化为nullptr，参数=窗口高度，宽度，窗口标题，窗口显示模式（null=窗口，不全屏），是否和窗口共享资源
  if (window == nullptr) {  // 如果窗口创建失败
    return false;           // 创建失败就false
  }
  appmap[window] = this;  // 在unordered map中，跟刚才创建的window搭配的application就是现在this指针所代表的application

  glfwSetFramebufferSizeCallback(
      window,
      fb_size_callback);  // 注册函数，注册回调函数为fb_size_callback()，实际调用了framebuffer_size_callback()，返回窗口的尺寸

  glfwMakeContextCurrent(
      window);  // 设置当前窗口的上下文环境，设置完之后，所有的OpenGL命令都会用来改变当前的上下文的状态信息
  glfwSwapInterval(1);  // 交换间隔表示交换缓冲区之前等待的帧数，默认情况下，交换间隔为0

  if (gl3wInit()) {
    std::cerr << "failed to initialize GL3W" << std::endl;
    return false;
  }

  // setup imgui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();  // 设置ImGui上下文
  auto &io = ImGui::GetIO();
  io.IniFilename = imgui_config_path;

  ImGui::StyleColorsClassic();  // 设置颜色风格

  ImGui_ImplGlfw_InitForOpenGL(window, true);  // Setup Platform/Renderer bindings
  ImGui_ImplOpenGL3_Init(glsl_version);

  return true;
}

void Application::run() {
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    draw_ui();

    ImGui::Render();

    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);

    // 设置背景色
    glClearColor(82.0 / 255.0, 82.0 / 255.0, 82.0 / 255.0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    draw_gl();

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
    usleep(5000);
  }
  // 释放资源
  free();
}

void Application::close() { glfwSetWindowShouldClose(window, 1); }

Eigen::Vector2i Application::framebuffer_size() {
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  return Eigen::Vector2i{width, height};
}

void Application::framebuffer_size_callback(const Eigen::Vector2i &size) {
  std::cout << "FB:" << size.transpose() << std::endl;
}

void Application::draw_ui() { ImGui::ShowDemoWindow(); }

void Application::draw_gl() {}

}  // namespace guik
