#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);

int main() {
  // 初始化GLFW
  glfwInit();

  // glfwWindowHint() - 配置GLFW
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);                    // 第一个参数代表选项的名称
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);                    // 第二个参数接受一个整型，用来设置这个选项的值
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);    // 告诉GLFW我们使用的是核心模式(Core-profile)，意味着我们只能使用OpenGL功能的一个子集

  // 创建一个窗口对象，这个窗口对象存放了所有和窗口相关的数据，而且会被GLFW的其他函数频繁地用到
  GLFWwindow* window = glfwCreateWindow(800, 600, "Learn_OpenGL", nullptr, nullptr);  // 宽，高，窗口名称，xx， xx - 函数将会返回一个GLFWwindow对象
  if (window == nullptr) {
    std::cout << "Failed to create GLFW Window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);     // 通知GLFW将我们窗口的上下文设置为当前线程的主上下文了

  // 当用户改变窗口的大小的时候，视口也应该被调整。所以将设置窗口维度的函数glViewport包装成回调函数
  // 注册这个函数，告诉GLFW我们希望每当窗口调整大小的时候调用这个函数
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

  // GLAD用来管理OpenGL的函数指针，在调用任何OpenGL的函数之前我们需要初始化GLAD
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {      // 给GLAD传入用来加载系统相关的OpenGL函数指针地址的函数，GLFW给我们的是glfwGetProcAddress
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  // 希望程序在我们主动关闭它之前不断绘制图像并能够接受用户输入；因此添加一个while渲染循环(Render Loop)，它能在我们让GLFW退出前一直保持运行
  while (!glfwWindowShouldClose(window)) {    // glfwWindowShouldClose() - 每次循环开始前检查一次GLFW是否被要求退出
    // 输入
    processInput(window);

    // 渲染
    glClear(GL_COLOR_BUFFER_BIT);

    // 检查并调用事件，交换缓冲
    glfwPollEvents();                         // 检查有没有触发事件（比如键盘输入、鼠标移动等）、更新窗口状态，并调用对应的回调函数（可以通过回调方法手动设置）
    glfwSwapBuffers(window);                  // 交换颜色缓冲（是储存着GLFW窗口每一个像素颜色值的大缓冲），它在这一迭代中被用来绘制，并且将会作为输出显示在屏幕上
  }

  glfwTerminate();                            // 释放/删除之前的分配的所有资源
  return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
  // 告诉OpenGL渲染窗口的尺寸大小，即视口(Viewport)，这样OpenGL才只能知道怎样根据窗口大小显示数据和坐标
  // glViewport() - 设置窗口的维度(Dimension)
  // 前两个参数控制窗口左下角的位置；第三个和第四个参数控制渲染窗口的宽度和高度（像素）
  glViewport(0, 0, width, height);
}

// 希望能够在GLFW中实现一些输入控制
void processInput(GLFWwindow* window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {    // 返回这个按键是否正在被按下，如果没有按下，glfwGetKey将会返回GLFW_RELEASE
    glfwSetWindowShouldClose(window, true);
  }
}