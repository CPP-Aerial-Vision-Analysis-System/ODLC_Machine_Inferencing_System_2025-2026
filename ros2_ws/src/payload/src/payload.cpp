// payload: servo controller (C++ port of payload/payload.py)
//
// Moves the bottle and beacon release servos via the MAVROS command service
// (MAV_CMD_DO_SET_SERVO). On startup it opens both servos once, mirroring the
// Python run_sequence(). Status messages are published to /mavros/statustext/send.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr int SERVO_BOTTLE = 9;       // AUX1 = Servo 9
constexpr int SERVO_BEACON = 10;      // AUX2 = Servo 10
constexpr int PULLEY_OPEN_BOTTLE = 1900;
constexpr int PULLEY_CLOSE_BOTTLE = 1400;
constexpr int PULLEY_OPEN_BEACON = 1900;
constexpr int PULLEY_CLOSE_BEACON = 1400;
}  // namespace

class ServoController : public rclcpp::Node
{
public:
  ServoController()
  : Node("servo_controller")
  {
    command_client_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    wait_for_services();

    status_pub_ = create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send", 10);
  }

  void run_sequence()
  {
    move_servo(SERVO_BOTTLE, PULLEY_OPEN_BOTTLE);
    move_servo(SERVO_BEACON, PULLEY_OPEN_BEACON);
  }

private:
  void wait_for_services()
  {
    while (!command_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "/mavros/cmd/command service not available, waiting...");
    }
    while (!set_mode_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "/mavros/set_mode service not available, waiting...");
    }
  }

  void move_servo(int channel, int pwm)
  {
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->broadcast = false;
    request->command = 183;  // MAV_CMD_DO_SET_SERVO
    request->confirmation = 0;
    request->param1 = static_cast<float>(channel);
    request->param2 = static_cast<float>(pwm);
    request->param3 = 0.0f;
    request->param4 = 0.0f;
    request->param5 = 0.0f;
    request->param6 = 0.0f;
    request->param7 = 0.0f;

    auto future = command_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Service call failed");
      return;
    }

    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(get_logger(), "[SERVO] Channel %d moved to %dus", channel, pwm);
      send_status("Servo " + std::to_string(channel) + " -> " + std::to_string(pwm));
    } else {
      RCLCPP_WARN(get_logger(), "[SERVO] Failed to move channel %d", channel);
      send_status("Servo " + std::to_string(channel) + " move FAILED");
    }
  }

  void send_status(const std::string & text, bool throttle = false)
  {
    const double now = this->now().seconds();
    if (!throttle || (now - last_status_time_ > status_interval_)) {
      mavros_msgs::msg::StatusText status_msg;
      status_msg.severity = 6;  // NOTICE
      status_msg.text = text;
      status_pub_->publish(status_msg);
      last_status_time_ = now;
    }
  }

  double last_status_time_{0.0};
  double status_interval_{5.0};

  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto servo_controller = std::make_shared<ServoController>();
  servo_controller->run_sequence();
  rclcpp::shutdown();
  return 0;
}
