#pragma once

#include <string>
#include <vector>

namespace franka_ros_controllers {

struct RobotSpecification {
  RobotSpecification() = default;
  RobotSpecification(const std::string& name, const std::vector<std::string>& joint_names)
      : name(name), joint_names(joint_names) {}

  std::string name{};
  std::vector<std::string> joint_names{};
};

}  // 