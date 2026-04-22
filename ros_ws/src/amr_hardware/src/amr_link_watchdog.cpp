#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr uint32_t COMM_FAULT_STARTUP_TIMEOUT = 1U << 0;
constexpr uint32_t COMM_FAULT_STALE_WHEEL_STATE = 1U << 1;
}

class AmrLinkWatchdog : public rclcpp::Node
{
public:
  AmrLinkWatchdog()
  : Node("amr_link_watchdog")
  {
    wheel_state_topic_ = declare_parameter<std::string>("wheel_state_topic", "/amr/wheel_state");
    startup_timeout_sec_ = declare_parameter<double>("startup_timeout_sec", 5.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 1.0);
    publish_period_sec_ = declare_parameter<double>("publish_period_sec", 0.5);

    comm_ok_pub_ = create_publisher<std_msgs::msg::Bool>("/amr/comm_ok", 10);
    comm_fault_pub_ = create_publisher<std_msgs::msg::Bool>("/amr/comm_fault", 10);
    comm_fault_mask_pub_ = create_publisher<std_msgs::msg::UInt32>("/amr/comm_fault_mask", 10);
    comm_status_pub_ = create_publisher<std_msgs::msg::String>("/amr/comm_status", 10);

    wheel_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      wheel_state_topic_,
      rclcpp::QoS(10).best_effort(),
      std::bind(&AmrLinkWatchdog::handle_wheel_state, this, std::placeholders::_1));

    start_time_ = now();
    last_wheel_state_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(publish_period_sec_)),
      std::bind(&AmrLinkWatchdog::publish_status, this));
  }

private:
  void handle_wheel_state(const sensor_msgs::msg::JointState::SharedPtr /*msg*/)
  {
    last_wheel_state_time_ = now();
  }

  void publish_status()
  {
    const auto t_now = now();
    const bool have_wheel_state = last_wheel_state_time_.nanoseconds() != 0;

    uint32_t fault_mask = 0U;
    std::string status = "stm_link_ok";

    if (!have_wheel_state) {
      const double since_start = (t_now - start_time_).seconds();
      if (since_start >= startup_timeout_sec_) {
        fault_mask |= COMM_FAULT_STARTUP_TIMEOUT;
        status = "startup_timeout_waiting_for_wheel_state";
      } else {
        status = "waiting_for_first_wheel_state";
      }
    } else {
      const double age = (t_now - last_wheel_state_time_).seconds();
      if (age >= stale_timeout_sec_) {
        fault_mask |= COMM_FAULT_STALE_WHEEL_STATE;
        status = "wheel_state_stale";
      }
    }

    const bool comm_ok = fault_mask == 0U && have_wheel_state;
    if (!last_comm_ok_valid_ || comm_ok != last_comm_ok_ || status != last_status_) {
      if (comm_ok) {
        RCLCPP_INFO(get_logger(), "STM link healthy: receiving %s", wheel_state_topic_.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "STM link unhealthy: %s", status.c_str());
      }
      last_comm_ok_ = comm_ok;
      last_comm_ok_valid_ = true;
      last_status_ = status;
    }

    std_msgs::msg::Bool comm_ok_msg;
    comm_ok_msg.data = comm_ok;
    comm_ok_pub_->publish(comm_ok_msg);

    std_msgs::msg::Bool comm_fault_msg;
    comm_fault_msg.data = !comm_ok;
    comm_fault_pub_->publish(comm_fault_msg);

    std_msgs::msg::UInt32 fault_mask_msg;
    fault_mask_msg.data = fault_mask;
    comm_fault_mask_pub_->publish(fault_mask_msg);

    std_msgs::msg::String status_msg;
    status_msg.data = status;
    comm_status_pub_->publish(status_msg);
  }

  std::string wheel_state_topic_;
  double startup_timeout_sec_{5.0};
  double stale_timeout_sec_{1.0};
  double publish_period_sec_{0.5};

  rclcpp::Time start_time_;
  rclcpp::Time last_wheel_state_time_;
  bool last_comm_ok_{false};
  bool last_comm_ok_valid_{false};
  std::string last_status_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr comm_ok_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr comm_fault_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr comm_fault_mask_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr comm_status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmrLinkWatchdog>());
  rclcpp::shutdown();
  return 0;
}
