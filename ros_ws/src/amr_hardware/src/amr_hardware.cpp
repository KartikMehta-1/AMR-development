#include "amr_hardware/amr_hardware.hpp"

#include <algorithm>
#include <chrono>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace amr_hardware {

hardware_interface::CallbackReturn AMRHardware::on_init(
    const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("amr_hardware"),
                 "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_joint_ = info_.joints[0].name;
  right_joint_ = info_.joints[1].name;

  left_cmd_topic_ = info_.hardware_parameters.count("left_cmd_topic")
                        ? info_.hardware_parameters.at("left_cmd_topic")
                        : std::string("/amr/wheel_cmd_left");
  right_cmd_topic_ = info_.hardware_parameters.count("right_cmd_topic")
                         ? info_.hardware_parameters.at("right_cmd_topic")
                         : std::string("/amr/wheel_cmd_right");
  state_topic_ = info_.hardware_parameters.count("state_topic")
                     ? info_.hardware_parameters.at("state_topic")
                     : std::string("/amr/wheel_state");

  hw_positions_ = std::vector<double>(2, 0.0);
  hw_velocities_ = std::vector<double>(2, 0.0);
  hw_commands_ = std::vector<double>(2, 0.0);

  ensure_ros();

  node_ = std::make_shared<rclcpp::Node>("amr_hardware");
  left_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32>(left_cmd_topic_, 10);
  right_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32>(right_cmd_topic_, 10);
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      state_topic_, 10,
      std::bind(&AMRHardware::handle_joint_state, this, std::placeholders::_1));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AMRHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(left_joint_, hardware_interface::HW_IF_POSITION,
                                &hw_positions_[0]);
  state_interfaces.emplace_back(left_joint_, hardware_interface::HW_IF_VELOCITY,
                                &hw_velocities_[0]);
  state_interfaces.emplace_back(right_joint_, hardware_interface::HW_IF_POSITION,
                                &hw_positions_[1]);
  state_interfaces.emplace_back(right_joint_, hardware_interface::HW_IF_VELOCITY,
                                &hw_velocities_[1]);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AMRHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(left_joint_, hardware_interface::HW_IF_VELOCITY,
                                  &hw_commands_[0]);
  command_interfaces.emplace_back(right_joint_, hardware_interface::HW_IF_VELOCITY,
                                  &hw_commands_[1]);
  return command_interfaces;
}

hardware_interface::CallbackReturn AMRHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  hw_positions_[0] = 0.0;
  hw_positions_[1] = 0.0;
  hw_velocities_[0] = 0.0;
  hw_velocities_[1] = 0.0;
  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;

  start_spinning();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AMRHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  stop_spinning();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AMRHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AMRHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!left_cmd_pub_ || !right_cmd_pub_) {
    return hardware_interface::return_type::ERROR;
  }

  std_msgs::msg::Float32 left_msg;
  std_msgs::msg::Float32 right_msg;
  left_msg.data = static_cast<float>(hw_commands_[0]);
  right_msg.data = static_cast<float>(hw_commands_[1]);

  left_cmd_pub_->publish(left_msg);
  right_cmd_pub_->publish(right_msg);

  return hardware_interface::return_type::OK;
}

void AMRHardware::handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == left_joint_) {
      if (i < msg->position.size()) {
        hw_positions_[0] = msg->position[i];
      }
      if (i < msg->velocity.size()) {
        hw_velocities_[0] = msg->velocity[i];
      }
    } else if (msg->name[i] == right_joint_) {
      if (i < msg->position.size()) {
        hw_positions_[1] = msg->position[i];
      }
      if (i < msg->velocity.size()) {
        hw_velocities_[1] = msg->velocity[i];
      }
    }
  }
}

void AMRHardware::ensure_ros() {
  if (!rclcpp::is_initialized()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

void AMRHardware::start_spinning() {
  if (spinning_.load()) {
    return;
  }
  spinning_.store(true);
  spin_thread_ = std::thread([this]() {
    rclcpp::Rate rate(200);
    while (rclcpp::ok() && spinning_.load()) {
      executor_->spin_some();
      rate.sleep();
    }
  });
}

void AMRHardware::stop_spinning() {
  if (!spinning_.load()) {
    return;
  }
  spinning_.store(false);
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
}

}  // namespace amr_hardware

PLUGINLIB_EXPORT_CLASS(amr_hardware::AMRHardware, hardware_interface::SystemInterface)
