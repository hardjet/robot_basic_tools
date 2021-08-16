#pragma once
namespace util {

bool publish_extrinsics(ros::NodeHandle& nh_, std::array<float, 6> ext, const std::string& to_frame,
                        const std::string& from_frame, const std::string& child_selected,
                        const std::string& parent_selected);

}  // namespace util