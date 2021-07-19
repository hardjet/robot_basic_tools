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
  void print_frames();

 public:
  void tf_static_callback(const tf2_msgs::TFMessage& msg);
//  void tf_callback(const geometry_msgs::TransformStamped& msg);

 private:
  ros::NodeHandle ros_nh_;
  ros::Subscriber tf_static_sub_;
//  ros::Subscriber tf_sub_;

  std::vector<geometry_msgs::TransformStamped> all_transforms_;
  std::set<std::string> all_frames_set_;
  std::vector<std::string> all_frames_vector_;
  std::vector<std::string> major_frames_;

  bool b_show_window_{false};
};

}
