#pragma once

// ros includes
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
// #include <franka_ros_controllers/controller_interface.h>
// #include <franka_ros_controllers/utils/robot_specification.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_srvs/Trigger.h>

// system includes
#include <memory>
#include <string>
#include <vector>

#include <franka_hw/franka_model_interface.h>

namespace franka_ros_controllers {

/// Container for the data defining the stop motion.
struct JointStopConfig {
  JointStopConfig() = delete;
  JointStopConfig(const double stiffness, const double damping, const double dq_stopped_threshold)
      : stiffness_m(stiffness), damping_m(damping), dq_stopped_threshold_m(dq_stopped_threshold) {}

  const double stiffness_m;
  const double damping_m;
  const double dq_stopped_threshold_m;
  double q_stopped_target_m{};
};

/// Controller impl class that uses a 2-zone approach (D and PD) to bring an arbitrary number of
/// Joints to rest. It first applies pure damping until the joints move very slow and then switches
/// to keeping the current position.
class JointStopController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface> {
 public:
  /**
   * @brief init Claims ros control hardware interfaces, parses all required parameters and sets up
   * services for testing if configured.
   *
   * @param[in] robot_hw  Pointer to the hardware class controling the Panda.
   * @param[in] node_handle A node handle in the controller namespace.
   *
   * @return True if successfull, false otherwise.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;

  /**
   * @brief starting Resets internal flags such that the stopping behavior is triggered.
   */
  void starting(const ros::Time&) override;

  /**
   * @brief update Computes the desired torques and commands them to the robot.
   */
  void update(const ros::Time&, const ros::Duration&) override;

  /**
   * @brief isAtRest Method to check whether the controlled robot is at rest/stopped with all
   * joints.
   *
   * @return True if all joints have come to rest, false otherwise.
   */
  bool isAtRest();

  /**
   * @brief jointStopTorque Computes the torques rendering both D and PD zones for stopping.
   * This method also updates the flags indicating whether joints are stopped (is_stopped_).
   *
   * @param[in] q Current joint position.
   * @param[in] dq Current joint velocity.
   * @param[in] config The configuration to stop this joint.
   * @param[out] is_stopped The flag is updated in this method and indicates whether the joint is
   * at rest.
   * @return The computed torque.
   */
  static double jointStopTorque(const double q,
                                const double dq,
                                JointStopConfig& config,
                                std::vector<bool>::reference is_stopped);

 private:
  const double default_stiffness_{100.0};
  const double default_damping_{10.0};
  std::string arm_id_;
  std::vector<std::string> joint_names_;
  bool verbose_{false};
  std::vector<bool> is_stopped_;
  std::vector<JointStopConfig> config_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  bool stop_in_progress_{false};

  // Optional service interface for testing (offered if testing param is true).
  ros::ServiceServer trigger_server_;
  ros::ServiceServer free_server_;
  bool stopTriggerCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool freeTriggerCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
};

}  // namespace franka_ros_controllers