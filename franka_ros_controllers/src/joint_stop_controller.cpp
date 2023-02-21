// Copyright (c) 2018 Franka Emika GmbH

// #include <franka_ros_controllers/controller_impl_factory.h>
#include <franka_ros_controllers/joint_stop_controller.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

// ros includes
#include <hardware_interface/joint_command_interface.h>
#include <std_srvs/Trigger.h>

// system includes
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

namespace franka_ros_controllers {

bool JointStopController::init(hardware_interface::RobotHW* robot_hw,
                             ros::NodeHandle& node_handle) {
  // moved from ctor
  std::string arm_id_;
  if (!node_handle.getParam("/robot_config/arm_id", arm_id_)) {
    ROS_ERROR("EffortJointTorqueController: Could not read parameter arm_id");
    return false;
  }
  // std::vector<std::string> joint_names_;
  if (!node_handle.getParam("/robot_config/joint_names", joint_names_) || joint_names_.size() != 7) {
    ROS_ERROR(
        "EffortJointTorqueController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  is_stopped_.resize(joint_names_.size());
  //

  std::vector<double> damping;
  if (!node_handle.getParam("/franka_ros_interface/joint_stop_controller/stop/damping", damping) || damping.size() != joint_names_.size()) {
    ROS_WARN(
        "JointStopController: Got insufficienct parameters for damping, using default %f for"
        "all joints",
        default_damping_);
    damping.resize(joint_names_.size());
    std::fill(damping.begin(), damping.end(), default_damping_);
  }
  // Check for invalid negative damping
  if (std::any_of(damping.begin(), damping.end(), [](double d) { return d < 0.0; })) {
    ROS_ERROR("JointStopController: Got negative damping. Aborting!");
    return false;
  }

  std::vector<double> stiffness;
  if (!node_handle.getParam("/franka_ros_interface/joint_stop_controller/stop/stiffness", stiffness) ||
      stiffness.size() != joint_names_.size()) {
    ROS_WARN(
        "JointStopController: Got insufficienct parameters for stiffness, using default %f for"
        "all joints",
        default_stiffness_);
    stiffness.resize(joint_names_.size());
    std::fill(stiffness.begin(), stiffness.end(), default_stiffness_);
  }
  // Check for invalid negative stiffness
  if (std::any_of(stiffness.begin(), stiffness.end(), [](double k) { return k < 0.0; })) {
    ROS_ERROR("JointStopController: Got negative stiffness. Aborting!");
    return false;
  }

  std::vector<double> dq_stopped_threshold;
  double default_dq_stopped_threshold(0.05);
  if (!node_handle.getParam("/franka_ros_interface/joint_stop_controller/stop/dq_stopped_threshold", dq_stopped_threshold) ||
      dq_stopped_threshold.size() != joint_names_.size()) {
    ROS_WARN(
        "JointStopController: Got insufficient parameters dq_stopped_threshold, using default:"
        "%f",
        default_dq_stopped_threshold);
    dq_stopped_threshold.resize(joint_names_.size());
    std::fill(dq_stopped_threshold.begin(), dq_stopped_threshold.end(),
              default_dq_stopped_threshold);
  }

  node_handle.getParam("/franka_ros_interface/joint_stop_controller/verbose", verbose_);

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("JointStopController: Error getting effort joint interface from hardware");
    return false;
  }

  for (const auto& name : joint_names_) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(name));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("JointStopController: Exception getting joint handle for joint "
                       << name << ": " << ex.what());
      return false;
    }
  }

  std::fill(is_stopped_.begin(), is_stopped_.end(), false);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    config_.emplace_back(stiffness.at(i), damping.at(i), std::abs(dq_stopped_threshold.at(i)));
  }

  bool testing = false;
  node_handle.getParam("/franka_ros_interface/joint_stop_controller/stop/testing", testing);
  if (testing) {
    trigger_server_ =
        node_handle.advertiseService(arm_id_ + "/stop", &JointStopController::stopTriggerCb, this);
    free_server_ =
        node_handle.advertiseService(arm_id_ + "/free", &JointStopController::freeTriggerCb, this);
    if (verbose_) {
      ROS_INFO(
          "JointStopController: Offering test services under %s/stop"
          "and %s/free",
          arm_id_.c_str(), arm_id_.c_str());
    }
  }

  if (verbose_) {
    ROS_INFO("JointStopController intialized for arm %s for joints:", arm_id_.c_str());
    for (auto& name : joint_names_) {
      ROS_INFO("%s", name.c_str());
    }
    ROS_INFO("JointStopController: Got paramerters damping:");
    for (auto& d : damping) {
      ROS_INFO("%f", d);
    }
    ROS_INFO("JointStopController: Got paramerters stiffness:");
    for (auto& k : stiffness) {
      ROS_INFO("%f", k);
    }
    ROS_INFO("JointStopController: Got paramerters dq_stopped_threshold:");
    for (auto& t : dq_stopped_threshold) {
      ROS_INFO("%f", t);
    }
    ROS_INFO("JointStopController: Offering stop trigger service at 'trigger_stop' ");
  }
  return true;
}

void JointStopController::starting(const ros::Time& /* time */) {
  
  stop_in_progress_ = true;
  std::fill(is_stopped_.begin(), is_stopped_.end(), false);
}

void JointStopController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    if (stop_in_progress_) {
      double tau_stop =
          jointStopTorque(joint_handles_.at(i).getPosition(), joint_handles_.at(i).getVelocity(),
                          config_.at(i), is_stopped_.at(i));
      if (is_stopped_.at(i) && verbose_) {
        ROS_INFO_THROTTLE(1, "%s Joint %zu is stopped", arm_id_.c_str(), i);
      }
      joint_handles_.at(i).setCommand(tau_stop);
    } else {
      joint_handles_.at(i).setCommand(0.0);
    }
  }

  if (verbose_) {
    ROS_INFO_THROTTLE(1, "JointStopController: stop in progress: %d", int(stop_in_progress_));
    ROS_INFO_THROTTLE(1, "JointStopController: is_stopped: ");
    for (size_t i = 0; i < is_stopped_.size(); ++i) {
      ROS_INFO_THROTTLE(1, "%s: %d", joint_names_.at(i).c_str(), int(bool(is_stopped_.at(i))));
    }
  }
}

bool JointStopController::isAtRest() {
  return not(std::any_of(is_stopped_.begin(), is_stopped_.end(), [](bool flag) { return !flag; }));
}

double JointStopController::jointStopTorque(const double q,
                                            const double dq,
                                            JointStopConfig& config,
                                            std::vector<bool>::reference is_stopped) {
  // Transition D-zone to PD-zone
  if (!is_stopped && std::abs(dq) < config.dq_stopped_threshold_m) {
    config.q_stopped_target_m = q;
    is_stopped = true;
  }
  if (!is_stopped) {
    return -config.damping_m * dq;  // D zone
  }
  return config.stiffness_m * (config.q_stopped_target_m - q) - config.damping_m * dq;  // PD zone
}

bool JointStopController::stopTriggerCb(std_srvs::Trigger::Request& /*req*/,
                                        std_srvs::Trigger::Response& res) {
  if (!stop_in_progress_) {
    starting(ros::Time::now());
    res.success = 1U;
    if (verbose_) {
      ROS_INFO("JointStopController: Stop triggered.");
    }
    return true;
  }
  if (verbose_) {
    ROS_WARN("JointStopController: Stop trigger declined as stop is already ongoing.");
  }
  res.success = 0U;
  res.message = "Stopping already in progress.";
  return true;
}

bool JointStopController::freeTriggerCb(std_srvs::Trigger::Request& /*req*/,
                                        std_srvs::Trigger::Response& res) {
  stop_in_progress_ = false;
  res.success = 1U;
  if (verbose_) {
    ROS_WARN("JointStopController: Freed robot motion.");
  }
  return true;
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::JointStopController,
                       controller_interface::ControllerBase)
