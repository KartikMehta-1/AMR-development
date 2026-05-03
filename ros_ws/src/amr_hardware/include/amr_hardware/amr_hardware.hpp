#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>

namespace amr_hardware {

class AMRHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AMRHardware)

  hardware_interface::return_type configure(
      const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

  std::string get_name() const override;
  hardware_interface::status get_status() const override;

private:
  void ensure_ros();
  void start_spinning();
  void stop_spinning();
  void handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::string left_joint_;
  std::string right_joint_;
  std::string left_cmd_topic_;
  std::string right_cmd_topic_;
  std::string state_topic_;
  double state_stale_timeout_sec_{0.5};

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> pending_positions_;
  std::vector<double> pending_velocities_;
  std::mutex state_mutex_;
  std::chrono::steady_clock::time_point last_state_time_{};
  bool have_state_{false};
  bool stale_command_logged_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;

  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spinning_{false};

  hardware_interface::HardwareInfo info_;
  hardware_interface::status status_{hardware_interface::status::UNKNOWN};
};

}  // namespace amr_hardware
