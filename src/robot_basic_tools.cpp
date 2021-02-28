// ui/opengl 相关
#include "imgui.h"
#include "portable-file-dialogs.h"

#include "glk/primitives/primitives.hpp"
#include "glk/drawble.hpp"
#include "glk/texture.hpp"
#include "glk/glsl_shader.hpp"
#include "glk/frame_buffer.hpp"
#include "glk/texture_renderer.hpp"

#include "guik/gl_canvas.hpp"
#include "guik/progress_modal.hpp"
#include "guik/camera_control.hpp"

#include "guik/projection_control.hpp"

#include "util/singleton.hpp"

// 传感器设备管理
#include "dev/sensor_manager.hpp"
#include "dev/april_board.hpp"

// 标定工具
#include "calibration/camera_laser_calib.hpp"

// ros相关
#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/init.h>
#include <ros/spinner.h>

#include "robot_basic_tools.hpp"

// debug
#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

bool RobotBasicTools::init(const char *window_name, const char *imgui_config_path, const Eigen::Vector2i &size,
                           const char *glsl_version) {
  if (!Application::init(window_name, imgui_config_path, size, glsl_version)) {
    return false;
  }

  is_show_imgui_demo = false;

  // 获取资源路径
  std::string package_path = ros::package::getPath("robot_basic_tools");
  dev::config_default_path = package_path + "/config";
  dev::data_default_path = package_path + "/data";

  right_clicked_pos.setZero();
  cur_mouse_pos.setZero();
  progress_ptr = std::make_unique<guik::ProgressModal>("progress modal");
  // 创建为单实例对象
  sensor_manager_ptr = util::Singleton<dev::SensorManager>::instance(nh);

  // 标定板
  april_board_ptr = std::make_shared<dev::AprilBoard>(dev::data_default_path);

  // 单线激光与相机标定
  cl_calib_ptr = std::make_unique<calibration::CamLaserCalib>(sensor_manager_ptr, april_board_ptr);

  // initialize the main OpenGL canvas
  main_canvas_ptr = std::make_unique<guik::GLCanvas>(dev::data_default_path, framebuffer_size());
  if (!main_canvas_ptr->ready()) {
    close();
  }

  return true;
}

void RobotBasicTools::draw_ui() {
  // draw main menu bar
  main_menu();

  // just for debug and development
  if (is_show_imgui_demo) {
    ImGui::ShowDemoWindow(&is_show_imgui_demo);
  }

  // show basic graph statistics and FPS
  // 设置窗口位置和大小
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  ImGui::SetNextWindowPos(ImVec2(float(width) - 150, 22), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(145, 100), ImGuiCond_Always);
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
  ImGui::Begin("##stats", nullptr, flags);

  // 计算3D坐标
  float depth = main_canvas_ptr->pick_depth(cur_mouse_pos);
  Eigen::Vector3f pos_3d{0, 0, 0};
  // +-1都不是有效的深度值
  if (fabs(std::fabs(depth) - 1.) > 1e-8) {
    pos_3d = main_canvas_ptr->unproject(cur_mouse_pos, depth);
  }
  // 显示
  ImGui::Text("mouse:(%d, %d)", cur_mouse_pos.x(), cur_mouse_pos.y());
  ImGui::Text("x:%.2f", pos_3d.x());
  ImGui::Text("y:%.2f", pos_3d.y());
  ImGui::Text("z:%.2f", pos_3d.z());

  // 判断热键是否按下
  ImGuiIO &io = ImGui::GetIO();
  // 按下大于0.5秒触发
  // alt: 342
  if (io.KeyAlt || io.KeysDownDuration[342] >= 0.5f) {
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 1.0f, 1.0f), "HotKey Activated!");
    is_hotkey_alt_pressed = true;
  } else {
    is_hotkey_alt_pressed = false;
  }

  ImGui::Text("FPS: %.3f fps", io.Framerate);
  ImGui::End();

  // todo message 模块重新做
  // ImGui::Begin("##messages", nullptr,
  //              ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground);
  // ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "%c %s", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3],
  //                    "running");
  //
  // // optimization progress messages
  // ImGui::BeginChild("##messages", ImVec2(100 + 30.0f, ImGui::GetFontSize() * 5), true,
  //                   ImGuiWindowFlags_AlwaysAutoResize);
  //
  // ImGui::Text("content--------\n----------");
  // ImGui::SetScrollHere();
  // ImGui::EndChild();
  // ImGui::End();

  // draw windows
  main_canvas_ptr->draw_ui();
  sensor_manager_ptr->draw_ui();
  april_board_ptr->draw_ui();
  cl_calib_ptr->draw_ui();

  context_menu();
  mouse_control();
}

void RobotBasicTools::draw_gl() {
  if (true) {
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    main_canvas_ptr->bind();

    // draw coordinate system
    main_canvas_ptr->shader->set_uniform("color_mode", 2);
    main_canvas_ptr->shader->set_uniform("model_matrix",
                                         (Eigen::UniformScaling<float>(1.0f) * Eigen::Isometry3f::Identity()).matrix());
    const auto &coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
    coord.draw(*main_canvas_ptr->shader);

    // draw grid
    main_canvas_ptr->shader->set_uniform("color_mode", 1);
    main_canvas_ptr->shader->set_uniform(
        "model_matrix",
        (Eigen::Translation3f(Eigen::Vector3f::UnitZ() * -0.02) * Eigen::Isometry3f::Identity()).matrix());
    main_canvas_ptr->shader->set_uniform("material_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
    const auto &grid = glk::Primitives::instance()->primitive(glk::Primitives::GRID);
    grid.draw(*main_canvas_ptr->shader);

    // let the windows draw something on the main canvas
    sensor_manager_ptr->draw_gl(*main_canvas_ptr->shader);

    // flush to the screen
    main_canvas_ptr->unbind();
    main_canvas_ptr->render_to_screen();

    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
  } else {
    // in case the optimization is going, show the last rendered image
    main_canvas_ptr->render_to_screen();
  }
}

void RobotBasicTools::free() {
  sensor_manager_ptr->free();
}

void RobotBasicTools::main_menu() {
  ImGui::BeginMainMenuBar();

  /*** File menu ***/
  // flags to open dialogs
  // this trick is necessary to open ImGUI popup modals from the menubar
  if (ImGui::BeginMenu("Setting")) {
    if (ImGui::MenuItem("AprilTag setting")) {
      april_board_ptr->show();
    }

    ImGui::Separator();

    if (ImGui::MenuItem("Quit")) {
      close();
    }

    ImGui::EndMenu();
  }

  /*** View menu ***/
  if (ImGui::BeginMenu("View")) {
    if (ImGui::MenuItem("Reset camera")) {
      main_canvas_ptr->reset_camera();
    }
    if (ImGui::MenuItem("Shader setting")) {
      main_canvas_ptr->show_shader_setting();
    }
    if (ImGui::MenuItem("Projection setting")) {
      main_canvas_ptr->show_projection_setting();
    }
    if (ImGui::MenuItem("Clear selections")) {
    }

    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Calibration")) {
    if (ImGui::MenuItem("camera & laser")) {
      cl_calib_ptr->show();
    }
    ImGui::EndMenu();
  }

  /*** Help menu ***/
  if (ImGui::BeginMenu("Help")) {
    if (ImGui::MenuItem("ImGuiDemo")) {
      is_show_imgui_demo = true;
    }

    if (ImGui::MenuItem("About")) {
    }

    ImGui::EndMenu();
  }

  ImGui::EndMainMenuBar();
}

void RobotBasicTools::mouse_control() {
  ImGuiIO &io = ImGui::GetIO();
  if (!io.WantCaptureMouse) {
    // let the main canvas handle the mouse input
    main_canvas_ptr->mouse_control();

    auto mouse_pos = ImGui::GetMousePos();
    // remember the right click position
    if (ImGui::IsMouseClicked(1)) {
      right_clicked_pos = Eigen::Vector2i(mouse_pos.x, mouse_pos.y);
    }

    // 记录当前鼠标位置，用于显示当前位置的3D坐标
    cur_mouse_pos = Eigen::Vector2i(mouse_pos.x, mouse_pos.y);
  }
}

void RobotBasicTools::context_menu() {
  if (ImGui::BeginPopupContextVoid("context menu")) {
    // pickup information of the right clicked object
    Eigen::Vector4i picked_info = main_canvas_ptr->pick_info(right_clicked_pos);
    int picked_type = picked_info[0];  // object type (point, vertex, edge, etc...)
    int picked_id = picked_info[1];    // object ID

    // calculate the 3D position of the right clicked pixel
    float depth = main_canvas_ptr->pick_depth(right_clicked_pos);
    Eigen::Vector3f pos_3d = main_canvas_ptr->unproject(right_clicked_pos, depth);

    ImGui::EndPopup();
  }
}

int main(int argc, char **argv) {
#ifdef USE_STACK_TRACE_LOGGER
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  std::cout << "Glog ON" << std::endl;
#endif

  ros::init(argc, argv, "robot_basic_tools");
  ros::NodeHandle nh("~");

  std::unique_ptr<guik::Application> app(new RobotBasicTools(nh));

  std::string glsl_version = "#version 330";
  // auto path = ros::package::getPath("robot_basic_tools") + "/imgui.ini";
  std::string path = "imgui.ini";

  if (!app->init("Robot Basic Tools", path.c_str(), Eigen::Vector2i(1280, 720), glsl_version.c_str())) {
    return 1;
  }

  // ros异步spinner
  ros::AsyncSpinner async_spinner(4);
  // 启动
  async_spinner.start();

  // 主逻辑
  app->run();

  // 关闭异步spinner线程
  async_spinner.stop();

  return 0;
}