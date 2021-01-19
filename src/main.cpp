#include <memory>
#include <imgui.h>
#include <portable-file-dialogs.h>

#include <glk/primitives/primitives.hpp>
#include <glk/drawble.hpp>
#include <glk/texture.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>

#include <guik/gl_canvas.hpp>
#include <guik/progress_modal.hpp>
#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>
#include <guik/projection_control.hpp>

#include <ros/package.h>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

class TestApplication : public guik::Application {
 public:
  TestApplication() : Application() {}

  ~TestApplication() override = default;

  /**
   * @brief initialize the application
   * @param window_name
   * @param size
   * @param glsl_version
   * @return true
   * @return false
   */
  bool init(const char *window_name, const Eigen::Vector2i &size, const char *glsl_version) override {
    if (!Application::init(window_name, size, glsl_version)) {
      return false;
    }

    show_imgui_demo = false;
    show_draw_config_window = true;

    right_clicked_pos.setZero();
    cur_mouse_pos.setZero();
    progress = std::make_unique<guik::ProgressModal>("progress modal");

    // initialize the main OpenGL canvas
    std::string package_path = ros::package::getPath("robot_basic_tools");
    std::string data_directory = package_path + "/data";

    main_canvas = std::make_unique<guik::GLCanvas>(data_directory, framebuffer_size());
    if (!main_canvas->ready()) {
      close();
    }

    return true;
  }

  /**
   * @brief draw ImGui-based UI
   *
   */
  void draw_ui() override {
    // draw main menu bar
    main_menu();

    // just for debug and development
    if (show_imgui_demo) {
      ImGui::ShowDemoWindow(&show_imgui_demo);
    }

    // show basic graph statistics and FPS
    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground;
    ImGui::Begin("##stats", nullptr, flags);

    // 计算3D坐标
    float depth = main_canvas->pick_depth(cur_mouse_pos);
    Eigen::Vector3f pos_3d{0, 0, 0};
    if (fabs(fabs(depth) - 1.) > 1e-8) {
      pos_3d = main_canvas->unproject(cur_mouse_pos, depth);
    }
    // 显示
    ImGui::Text("(%d, %d)-(%.2f, %.2f, %.2f)", cur_mouse_pos.x(), cur_mouse_pos.y(), pos_3d.x(), pos_3d.y(),
                pos_3d.z());

    // 判断热键是否按下
    ImGuiIO &io = ImGui::GetIO();
    // 按下大于0.5秒触发
    // alt: 342
    if (io.KeyAlt || io.KeysDownDuration[342] >= 0.5f) {
      ImGui::TextColored(ImVec4(1.0f, 0.0f, 1.0f, 1.0f), "HotKey Activated! %lu selected", selected_id.size());
      hotkey_press = true;
    } else {
      hotkey_press = false;
    }

    ImGui::Text("\nFPS: %.3f fps", io.Framerate);
    ImGui::End();

    ImGui::Begin("##optimization progress", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground);
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "%c %s", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3],
                       "optimizing");

    // optimization progress messages
    ImGui::BeginChild("##messages", ImVec2(20 + 30.0f, ImGui::GetFontSize() * 32), true,
                      ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("content------------------");
    ImGui::SetScrollHere();
    ImGui::EndChild();

    ImGui::End();

    // draw windows
    main_canvas->draw_ui();

    context_menu();
    mouse_control();
  }

  /**
   * @brief draw OpenGL related stuffs on the main canvas
   *
   */
  void draw_gl() override {
    if (true) {
      glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

      main_canvas->bind();

      // draw coordinate system
      main_canvas->shader->set_uniform("color_mode", 2);
      main_canvas->shader->set_uniform("model_matrix",
                                       (Eigen::UniformScaling<float>(3.0f) * Eigen::Isometry3f::Identity()).matrix());
      const auto &coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
      coord.draw(*main_canvas->shader);

      // draw grid
      main_canvas->shader->set_uniform("color_mode", 1);
      main_canvas->shader->set_uniform(
          "model_matrix",
          (Eigen::Translation3f(Eigen::Vector3f::UnitZ() * -0.02) * Eigen::Isometry3f::Identity()).matrix());
      main_canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
      const auto &grid = glk::Primitives::instance()->primitive(glk::Primitives::GRID);
      grid.draw(*main_canvas->shader);

      // let the windows draw something on the main canvas

      // flush to the screen
      main_canvas->unbind();
      main_canvas->render_to_screen();

      glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
    } else {
      // in case the optimization is going, show the last rendered image
      main_canvas->render_to_screen();
    }
  }

  /**
   * @brief frame buffer size change callback
   *
   * @param size
   */
  void framebuffer_size_callback(const Eigen::Vector2i &size) override { main_canvas->set_size(size); }

 private:
  /**
   * @brief draw main menu
   *
   */
  void main_menu() {
    ImGui::BeginMainMenuBar();

    /*** File menu ***/
    // flags to open dialogs
    // this trick is necessary to open ImGUI popup modals from the menubar
    bool open_map_only_dialog = false;
    bool open_deepblue_map_dialog = false;
    bool merge_map_dialog = false;
    bool save_map_dialog = false;
    bool save_nav_data_dialog = false;
    bool export_map_dialog = false;
    bool pcd2png_dialog = false;
    if (ImGui::BeginMenu("File")) {
      if (ImGui::BeginMenu("Open")) {
        if (ImGui::BeginMenu("Load map")) {
          if (ImGui::MenuItem("map")) {
            open_map_only_dialog = true;
          }
          if (ImGui::MenuItem("DeepBlue map")) {
            open_deepblue_map_dialog = true;
          }
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Merge map")) {
          if (ImGui::MenuItem("map")) {
            merge_map_dialog = true;
          }
          ImGui::EndMenu();
        }
        ImGui::EndMenu();
      }

      if (ImGui::BeginMenu("Save")) {
        if (ImGui::MenuItem("Save map data")) {
          save_map_dialog = true;
        }
        if (ImGui::MenuItem("Save nav data")) {
          save_nav_data_dialog = true;
        }

        ImGui::Separator();
        if (ImGui::MenuItem("nav setting")) {
        }
        ImGui::EndMenu();
      }

      if (ImGui::BeginMenu("Export")) {
        if (ImGui::MenuItem("Export PointCloud")) {
          export_map_dialog = true;
        }

        ImGui::Separator();
        if (ImGui::MenuItem("export setting")) {
        }
        ImGui::Separator();
        if (ImGui::MenuItem("pcd2png")) {
          pcd2png_dialog = true;
        }
        ImGui::EndMenu();
      }

      ImGui::Separator();

      if (ImGui::MenuItem("Quit")) {
        close();
      }

      ImGui::EndMenu();
    }

    // open dialogs
    // open_map_only_data(open_map_only_dialog);
    // open_deepblue_map_data(open_deepblue_map_dialog);
    // merge_map_data(merge_map_dialog);
    // save_map_data(save_map_dialog);
    // save_nav_data(save_nav_data_dialog);
    // export_pointcloud(export_map_dialog);
    // pcd2png(pcd2png_dialog);

    /*** View menu ***/
    if (ImGui::BeginMenu("View")) {
      if (ImGui::MenuItem("Reset camera")) {
        main_canvas->reset_camera();
      }
      if (ImGui::MenuItem("Projection setting")) {
        main_canvas->show_projection_setting();
      }
      if (ImGui::MenuItem("Graph rendering setting")) {
        show_draw_config_window = true;
      }
      if (ImGui::MenuItem("Clear selections")) {
      }

      ImGui::EndMenu();
    }

    /*** Help menu ***/
    bool show_version = false;
    if (ImGui::BeginMenu("Help")) {
      if (ImGui::MenuItem("ImGuiDemo")) {
        show_imgui_demo = true;
      }

      if (ImGui::MenuItem("About")) {
        show_version = true;
      }

      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }

 private:
  /**
   * @brief handling mouse input
   */
  void mouse_control() {
    ImGuiIO &io = ImGui::GetIO();
    if (!io.WantCaptureMouse) {
      // let the main canvas handle the mouse input
      main_canvas->mouse_control();

      auto mouse_pos = ImGui::GetMousePos();
      // remember the right click position
      if (ImGui::IsMouseClicked(1)) {
        right_clicked_pos = Eigen::Vector2i(mouse_pos.x, mouse_pos.y);
      }

      // 记录当前鼠标位置，用于显示当前位置的3D坐标
      cur_mouse_pos = Eigen::Vector2i(mouse_pos.x, mouse_pos.y);
    }
  }

  /**
   * @brief context menu
   */
  void context_menu() {
    if (ImGui::BeginPopupContextVoid("context menu")) {
      // pickup information of the right clicked object
      Eigen::Vector4i picked_info = main_canvas->pick_info(right_clicked_pos);
      int picked_type = picked_info[0];  // object type (point, vertex, edge, etc...)
      int picked_id = picked_info[1];    // object ID

      // calculate the 3D position of the right clicked pixel
      float depth = main_canvas->pick_depth(right_clicked_pos);
      Eigen::Vector3f pos_3d = main_canvas->unproject(right_clicked_pos, depth);

      ImGui::EndPopup();
    }
  }

 private:
  // DrawFlags draw_flags;
  Eigen::Vector2i right_clicked_pos;
  Eigen::Vector2i cur_mouse_pos;

  std::unique_ptr<guik::GLCanvas> main_canvas;
  std::unique_ptr<guik::ProgressModal> progress;

  // 热键标记(ALT)
  bool hotkey_press = false;
  std::set<int> selected_id;

  bool show_imgui_demo{};
  bool show_draw_config_window{};
};

int main(int argc, char **argv) {
#ifdef USE_STACK_TRACE_LOGGER
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  std::cout << "Glog ON" << std::endl;
#endif

  std::unique_ptr<guik::Application> app(new TestApplication());

  std::string glsl_version = "#version 330";

  auto path = ros::package::getPath("robot_basic_tools") + "/imgui.ini";
  std::cout << "imgui path:" << path << std::endl;

  // ImGuiIO &io = ImGui::GetIO();
  // io.IniFilename = "/home/anson/catkin_map/src/robot_basic_tools/imgui.ini";
  // std::cout << "imgui path:" << io.IniFilename << std::endl;
  if (!app->init("Robot Basic Tools", Eigen::Vector2i(1280, 720), glsl_version.c_str())) {
    return 1;
  }

  app->run();

  return 0;
}