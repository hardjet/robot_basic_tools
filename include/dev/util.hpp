#pragma once

namespace dev {


void help_marker(const char* desc);

void get_topic_name_from_list(const std::string& target_topic_type, std::vector<std::string>& candidates);

void show_pfd_info(const std::string& title, const std::string& msg);

}  // namespace dev
