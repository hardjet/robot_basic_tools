#include <ros/master.h>

#include "imgui.h"
#include "portable-file-dialogs.h"
#include "dev/util.hpp"

namespace dev {

void help_marker(const char* desc) {
  ImGui::TextDisabled("(?)");
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextUnformatted(desc);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void get_topic_name_from_list(const std::string& target_topic_type, std::vector<std::string>& candidates) {
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  candidates.clear();

  for (auto& info : master_topics) {
    if (info.datatype == target_topic_type) {
      candidates.push_back(info.name);
    }
  }
}

void show_pfd_info(const std::string& title, const std::string& msg) {
  pfd::message message(title, msg, pfd::choice::ok);
  while (!message.ready()) {
    usleep(1000);
  }
}

}  // namespace dev