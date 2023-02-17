#pragma once

#include <franka_ros_controllers/controller_interface.h>
#include <franka_ros_controllers/utils/abstract_factory.h>
#include <franka_ros_controllers/utils/robot_specification.h>

#include <vector>

namespace franka_ros_controllers {

using ControllerImplFactory =
    AbstractFactory<ControllerInterface, std::string, std::vector<RobotSpecification>>;

}  // namespace franka_ros_controllers