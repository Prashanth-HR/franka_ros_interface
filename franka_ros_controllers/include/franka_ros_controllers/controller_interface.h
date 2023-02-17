// Copyright (c) 2018 Franka Emika GmbH
#pragma once

#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_ros_controllers {

class ControllerInterface;
using ControllerInterfacePtr = std::shared_ptr<franka_ros_controllers::ControllerInterface>;

/**
 * A base class interface for controller implementations of various types.
 */
class ControllerInterface {
 public:
  virtual ~ControllerInterface() = default;

  /**
   * Initializes the controller implementation, e.g. by setting up interfaces and parsing
   * parameters. This method has to be called before operation.
   *
   * @param[in] robot_hw A pointer to the ros_control hardware class to control.
   * @param[in] node_handle A node handle to parse parameters, etc.
   *
   * @return true if successfull, false otherwise.
   */
  virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) = 0;

  /**
   * Resets internal states such that the controller can start running. This method must be realtime
   * capable.
   */
  virtual void starting(const ros::Time& /*time*/) = 0;

  /**
   * Computes on update time step of the controller and communicates the results.
   *
   * @param[in] period The time between consecutive calls to update().
   */
  virtual void update(const ros::Time& /*time*/, const ros::Duration& period) = 0;

  /**
   * Handles the situation when the controller is stopped (if necessary).
   */
  virtual void stopping(const ros::Time& /*time*/) {}

  /**
   * Checks whether the controlled robot is at rest/stopped with all joints.
   *
   * @return True if all joints have come to rest, false otherwise.
   */
  virtual bool isAtRest() { return false; }
};

}  // namespace franka_ros_controllers