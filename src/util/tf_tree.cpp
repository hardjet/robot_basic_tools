#include "imgui.h"

#include "util/tf_tree.hpp"

namespace util {

TfTree::TfTree(const ros::NodeHandle& nh) : ros_nh_(nh) {
  printf("----- TfTree::TfTree() ..... calling constructor\n");
  tf_static_sub_ = ros_nh_.subscribe("/tf_static", 1, &TfTree::tf_static_callback, this);
//  tf_sub_ = ros_nh_.subscribe("/tf", 1, &TfTree::tf_callback, this);
}

void TfTree::tf_static_callback(const tf2_msgs::TFMessage& msg) {
  printf("----- TfTree::tf_static_callback() ..... calling\n");

//  std::cout << "size : " << msg.transforms.size() << std::endl;
//  std::cout << msg << std::endl;

  geometry_msgs::TransformStamped current_tf;
  current_tf.header = msg.transforms[0].header;
  current_tf.child_frame_id = msg.transforms[0].child_frame_id;
  current_tf.transform = msg.transforms[0].transform;

  // transform关系不会重复，所以直接插入到vector
  all_transforms_.push_back(current_tf);

  // 构成一组transform关系的两个frame有可能会重复，所以要靠set去筛选
  std::pair<std::set<std::string>::iterator, bool> feedback_parent = all_frames_set_.insert(msg.transforms[0].header.frame_id);
  std::pair<std::set<std::string>::iterator, bool> feedback_child = all_frames_set_.insert(msg.transforms[0].child_frame_id);

  if (feedback_parent.second) {
    all_frames_vector_.push_back(msg.transforms[0].header.frame_id);
    if (msg.transforms[0].header.frame_id.find("/base") != std::string::npos) {
      major_frames_.push_back(msg.transforms[0].header.frame_id);
    }
  }

  if (feedback_child.second) {
    all_frames_vector_.push_back(msg.transforms[0].child_frame_id);
    if (msg.transforms[0].child_frame_id.find("/base") != std::string::npos) {
      major_frames_.push_back(msg.transforms[0].child_frame_id);
    }
  }
}

//void TfTree::tf_callback(const & msg) {
//}

void TfTree::show() {
  printf("----- TfTree::show() ..... calling\n");
  b_show_window_ = true;
}

void TfTree::draw_ui() {
  if (!b_show_window_) {
    return;
  }

  ImGui::SetNextWindowPos(ImVec2(210, 22), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(200, 300), ImGuiCond_FirstUseEver);
  ImGui::Begin("TF Tree", &b_show_window_, ImGuiWindowFlags_None);

  if (ImGui::TreeNode("/tf_static")) {
    for (int i = 0; i < all_transforms_.size(); ++i) {
      if (i == 0)
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);

      if (ImGui::TreeNode((void*)(intptr_t)i, "Transform %d", i)) {
        ImGui::Text("- header:");
        ImGui::Text("\t- frame_id: %s", all_transforms_[i].header.frame_id.c_str());
        ImGui::Text("\t- seq: %d", all_transforms_[i].header.seq);
        ImGui::Text("\t- stamp: %u", all_transforms_[i].header.stamp.sec);
        ImGui::Text("- child_frame_id: %s", all_transforms_[i].child_frame_id.c_str());
        ImGui::Text("- transform:");
        ImGui::Text("\t- translation:");
        ImGui::Text("\t\t- x: %f", all_transforms_[i].transform.translation.x);
        ImGui::Text("\t\t- y: %f", all_transforms_[i].transform.translation.y);
        ImGui::Text("\t\t- z: %f", all_transforms_[i].transform.translation.z);
        ImGui::Text("\t- rotation:");
        ImGui::Text("\t\t- x: %f", all_transforms_[i].transform.rotation.x);
        ImGui::Text("\t\t- y: %f", all_transforms_[i].transform.rotation.y);
        ImGui::Text("\t\t- z: %f", all_transforms_[i].transform.rotation.z);
        ImGui::Text("\t\t- w: %f", all_transforms_[i].transform.rotation.w);
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }
  ImGui::End();
}

std::vector<std::string> TfTree::get_major_frames() {
  return major_frames_;
}

void TfTree::print_frames() {
  printf("----- ALL FRAMES -----\n");
  int count = 0;
  for (auto& elem : all_frames_vector_) {
    printf("----- ----- %d : %s\n", count, elem.c_str());
    count++;
  }

  printf("----- MAJOR FRAMES -----\n");
  count = 0;
  for (auto& elem : major_frames_) {
    printf("----- ----- %d : %s\n", count, elem.c_str());
    count++;
  }
}

}   // namespace util


