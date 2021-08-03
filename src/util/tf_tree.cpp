#include "imgui.h"

#include "util/tf_tree.hpp"

namespace util {

TfTree::TfTree(const ros::NodeHandle& nh) : ros_nh_(nh) {
  //  printf("----- TfTree::TfTree() ..... calling constructor\n");
  tf_static_sub_ = ros_nh_.subscribe("/tf_static", 1, &TfTree::tf_static_callback, this);
  tf_sub_ = ros_nh_.subscribe("/tf", 1, &TfTree::tf_callback, this);
}

void TfTree::tf_static_callback(const tf2_msgs::TFMessage& msg) {
  geometry_msgs::TransformStamped current_tf;
  current_tf.header = msg.transforms[0].header;
  current_tf.child_frame_id = msg.transforms[0].child_frame_id;
  current_tf.transform = msg.transforms[0].transform;

  // transform关系不会重复，所以直接插入到vector
  tfs_transforms_.push_back(current_tf);

  // 构成一组transform关系的两个frame有可能会重复，所以要靠set去筛选
  std::pair<std::set<std::string>::iterator, bool> feedback_parent =
      tfs_frames_set_.insert(msg.transforms[0].header.frame_id);
  std::pair<std::set<std::string>::iterator, bool> feedback_child =
      tfs_frames_set_.insert(msg.transforms[0].child_frame_id);

  if (feedback_parent.second) {
    tfs_frames_vector_.push_back(msg.transforms[0].header.frame_id);
    if (msg.transforms[0].header.frame_id.find("/base") != std::string::npos) {
      tfs_major_frames_.push_back(msg.transforms[0].header.frame_id);
    }
  }

  if (feedback_child.second) {
    tfs_frames_vector_.push_back(msg.transforms[0].child_frame_id);
    if (msg.transforms[0].child_frame_id.find("/base") != std::string::npos) {
      tfs_major_frames_.push_back(msg.transforms[0].child_frame_id);
    }
  }
}

void TfTree::tf_callback(const tf2_msgs::TFMessage& msg) {
  std::pair<std::string, std::string> current_frame_pair{msg.transforms[0].header.frame_id,
                                                         msg.transforms[0].child_frame_id};

  if (tf_frame_pairs_.insert(current_frame_pair).second) {
    geometry_msgs::TransformStamped current_tf;
    current_tf.header = msg.transforms[0].header;
    current_tf.child_frame_id = msg.transforms[0].child_frame_id;
    current_tf.transform = msg.transforms[0].transform;
    tf_transforms_.push_back(current_tf);
  } else {
    for (auto tf : tf_transforms_) {
      if (std::strcmp(tf.header.frame_id.c_str(), msg.transforms[0].header.frame_id.c_str()) == 0 &&
          std::strcmp(tf.child_frame_id.c_str(), msg.transforms[0].child_frame_id.c_str()) == 0) {
        tf.transform = msg.transforms[0].transform;
        tf.header = msg.transforms[0].header;
        tf.child_frame_id = msg.transforms[0].child_frame_id;
        break;
      }
    }
  }

  if (tf_frames_set_.insert(msg.transforms[0].header.frame_id).second) {
    tf_frames_vector_.push_back(msg.transforms[0].header.frame_id);
    if (msg.transforms[0].header.frame_id.find("base") != std::string::npos) {
      tf_major_frames_.push_back(msg.transforms[0].header.frame_id);
    }
  }
  if (tf_frames_set_.insert(msg.transforms[0].child_frame_id).second) {
    tf_frames_vector_.push_back(msg.transforms[0].child_frame_id);
    if (msg.transforms[0].child_frame_id.find("base") != std::string::npos) {
      tf_major_frames_.push_back(msg.transforms[0].child_frame_id);
    }
  }
}

void TfTree::show() { b_show_window_ = true; }

void TfTree::draw_ui() {
  if (!b_show_window_) {
    return;
  }

  ImGui::SetNextWindowPos(ImVec2(210, 22), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(200, 300), ImGuiCond_FirstUseEver);
  ImGui::Begin("TF Tree", &b_show_window_, window_flags_);

  if (ImGui::TreeNode("/tf_static")) {
    for (int i = 0; i < tfs_transforms_.size(); ++i) {
      if (i == 0) ImGui::SetNextItemOpen(true, ImGuiCond_Once);

      if (ImGui::TreeNode((void*)(intptr_t)i, "Transform %d", i)) {
        ImGui::Text("- header:");
        ImGui::Text("\t- frame_id: %s", tfs_transforms_[i].header.frame_id.c_str());
        ImGui::Text("\t- seq: %d", tfs_transforms_[i].header.seq);
        ImGui::Text("\t- stamp: %u", tfs_transforms_[i].header.stamp.sec);
        ImGui::Text("- child_frame_id: %s", tfs_transforms_[i].child_frame_id.c_str());
        ImGui::Text("- transform:");
        ImGui::Text("\t- translation:");
        ImGui::Text("\t\t- x: %f", tfs_transforms_[i].transform.translation.x);
        ImGui::Text("\t\t- y: %f", tfs_transforms_[i].transform.translation.y);
        ImGui::Text("\t\t- z: %f", tfs_transforms_[i].transform.translation.z);
        ImGui::Text("\t- rotation:");
        ImGui::Text("\t\t- x: %f", tfs_transforms_[i].transform.rotation.x);
        ImGui::Text("\t\t- y: %f", tfs_transforms_[i].transform.rotation.y);
        ImGui::Text("\t\t- z: %f", tfs_transforms_[i].transform.rotation.z);
        ImGui::Text("\t\t- w: %f", tfs_transforms_[i].transform.rotation.w);
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("/tf")) {
    for (int i = 0; i < tf_transforms_.size(); ++i) {
      if (i == 0) ImGui::SetNextItemOpen(true, ImGuiCond_Once);

      if (ImGui::TreeNode((void*)(intptr_t)i, "Transform %d", i)) {
        ImGui::Text("- header:");
        ImGui::Text("\t- frame_id: %s", tf_transforms_[i].header.frame_id.c_str());
        ImGui::Text("\t- seq: %d", tf_transforms_[i].header.seq);
        ImGui::Text("\t- stamp: %u", tf_transforms_[i].header.stamp.sec);
        ImGui::Text("- child_frame_id: %s", tf_transforms_[i].child_frame_id.c_str());
        ImGui::Text("- transform:");
        ImGui::Text("\t- translation:");
        ImGui::Text("\t\t- x: %f", tf_transforms_[i].transform.translation.x);
        ImGui::Text("\t\t- y: %f", tf_transforms_[i].transform.translation.y);
        ImGui::Text("\t\t- z: %f", tf_transforms_[i].transform.translation.z);
        ImGui::Text("\t- rotation:");
        ImGui::Text("\t\t- x: %f", tf_transforms_[i].transform.rotation.x);
        ImGui::Text("\t\t- y: %f", tf_transforms_[i].transform.rotation.y);
        ImGui::Text("\t\t- z: %f", tf_transforms_[i].transform.rotation.z);
        ImGui::Text("\t\t- w: %f", tf_transforms_[i].transform.rotation.w);
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }
  ImGui::End();
}

std::vector<std::string> TfTree::get_major_frames() { return tfs_major_frames_; }

std::vector<std::string> TfTree::get_tf_major_frames() {
  if (tf_major_frames_.empty()) {
    std::vector<std::string> tmp_vector;
    tmp_vector.emplace_back("Empty tf ... Please check!");
    return tmp_vector;
  } else {
    return tf_major_frames_;
  }
}

std::vector<geometry_msgs::TransformStamped> TfTree::get_tf_transforms() { return tf_transforms_; }

void TfTree::print_frames() {
  printf("----- TF_STATIC FRAMES -----\n");
  int count = 0;
  for (auto& elem : tfs_frames_vector_) {
    printf("----- ----- %d : %s\n", count, elem.c_str());
    count++;
  }

  printf("----- TF_STATIC MAJOR FRAMES -----\n");
  count = 0;
  for (auto& elem : tfs_major_frames_) {
    printf("----- ----- %d : %s\n", count, elem.c_str());
    count++;
  }
}

}  // namespace util
