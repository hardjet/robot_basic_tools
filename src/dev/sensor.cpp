#include "imgui.h"
#include "dev/sensor.hpp"

namespace dev {

void Sensor::show() { is_show_window_ = true;}

void Sensor::draw_status() {
  if (is_online_) {
    ImGui::TextColored(ImVec4(COLOR_ONLINE[0], COLOR_ONLINE[1], COLOR_ONLINE[2], 1.0f), "·");
  } else {
    ImGui::TextColored(ImVec4(COLOR_OFFLINE[0], COLOR_OFFLINE[1], COLOR_OFFLINE[2], 1.0f), "·");
  }
}

}  // namespace dev