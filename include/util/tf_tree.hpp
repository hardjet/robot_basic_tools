#pragma once

#include <iostream>
#include <vector>
#include <set>

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

namespace util {

class TfTree {
 public:
  explicit TfTree(const ros::NodeHandle& nh);
  void show();
  void draw_ui();
  std::vector<std::string> get_major_frames();
  std::vector<std::string> get_tf_major_frames();
  std::vector<geometry_msgs::TransformStamped> get_tf_transforms();
  void print_frames();

 public:
  void tf_static_callback(const tf2_msgs::TFMessage& msg);
  void tf_callback(const tf2_msgs::TFMessage& msg);

 private:
  ros::NodeHandle ros_nh_;
  ros::Subscriber tf_static_sub_;
  ros::Subscriber tf_sub_;

  std::vector<geometry_msgs::TransformStamped> tfs_transforms_;
  std::set<std::string> tfs_frames_set_;
  std::vector<std::string> tfs_frames_vector_;
  std::vector<std::string> tfs_major_frames_;

  std::vector<geometry_msgs::TransformStamped> tf_transforms_;
  std::set<std::pair<std::string, std::string>> tf_frame_pairs_;
  std::set<std::string> tf_frames_set_;
  std::vector<std::string> tf_frames_vector_;
  std::vector<std::string> tf_major_frames_;

  ImGuiWindowFlags window_flags_ = 0;
  bool b_show_window_{false};
};

}
