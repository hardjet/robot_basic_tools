#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include "util/extrinsics_publisher.hpp"
#include "algorithm/util.h"
#include "robot_basic_tools/Extrinsic.h"

namespace util {
bool publish_extrinsics(ros::NodeHandle& nh_, std::array<float, 6> ext,
                        const std::string& child_true, const std::string& parent_true,
                        const std::string& child_selected, const std::string& parent_selected) {

  ros::ServiceClient client = nh_.serviceClient<robot_basic_tools::Extrinsic>("/calibrator_listener/sensor_extrinsic");

  geometry_msgs::TransformStamped ext_msg;
  tf::Quaternion q = tf::createQuaternionFromRPY(DEG2RAD_RBT(ext[3]), DEG2RAD_RBT(ext[4]), DEG2RAD_RBT(ext[5]));
  tf::Vector3 t = tf::Vector3(ext[0], ext[1], ext[2]);
  tf::transformStampedTFToMsg(tf::StampedTransform(tf::Transform(q, t),
                                                   ros::Time::now(),
                                                   "/" + parent_true,
                                                   "/" + child_true), ext_msg);
//  printf("roll = 0, pitch = 0, yaw = 0\n");
//  printf("q: [%f, %f, %f, %f]\n", ext_msg.transform.rotation.x, ext_msg.transform.rotation.y, ext_msg.transform.rotation.z, ext_msg.transform.rotation.w);
//  printf("t: [%f, %f, %f]\n", ext_msg.transform.translation.x, ext_msg.transform.translation.y, ext_msg.transform.translation.z);

  robot_basic_tools::Extrinsic output;
  output.request.tfs = ext_msg;
  output.request.parent_selected = parent_selected;
  output.request.child_selected = child_selected;

  if (client.call(output)) {
    ROS_INFO("Response from server: %d", output.response.feedback);
  }
  else {
    ROS_ERROR("Failed to call service sensor_extrinsic");
    return false;
  }
  return true;
}
}