// mapping: mission camera trigger (C++ port of mapping/do_digi_cam_trigger.py)
//
// Starts a 1 Hz /camera/trigger pulse when the autopilot reports a DigiCamCtrl
// mission item (via StatusText), and stops it once the mission reaches the
// buffer waypoint published by waypoint_manager. buffer_wp is fetched via the
// standard GetParameters service (inlined here, replacing the Python
// wp_sender.ParameterManager import).

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"

using namespace std::chrono_literals;

class MissionCameraTrigger : public rclcpp::Node
{
public:
  MissionCameraTrigger()
  : Node("mission_camera_trigger")
  {
    statustext_sub_ = create_subscription<mavros_msgs::msg::StatusText>(
      "/mavros/statustext/recv", rclcpp::SensorDataQoS(),
      [this](mavros_msgs::msg::StatusText::SharedPtr m) { statustext_callback(m); });
    reached_sub_ = create_subscription<mavros_msgs::msg::WaypointReached>(
      "/mavros/mission/reached", 1,
      [this](mavros_msgs::msg::WaypointReached::SharedPtr m) { update_waypoint_reached(m); });
    parameter_event_sub_ = create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      [this](rcl_interfaces::msg::ParameterEvent::SharedPtr m) { parameter_event_cb(m); });

    status_publisher_ =
      create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send", 10);
    camera_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("/camera/trigger", 10);

    waypoint_param_client_ =
      create_client<rcl_interfaces::srv::GetParameters>("waypoint_manager/get_parameters");

    fetch_mission_indices();
  }

private:
  void fetch_mission_indices()
  {
    if (!waypoint_param_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Could not fetch buffer_wp yet, using default -1");
      return;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"buffer_wp"};
    waypoint_param_client_->async_send_request(
      request,
      [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
        auto response = future.get();
        if (response->values.empty()) {
          RCLCPP_WARN(get_logger(), "Could not fetch buffer_wp, using default -1");
          buffer_wp_ = -1;
          return;
        }
        buffer_wp_ = static_cast<int>(response->values[0].integer_value);
      });
  }

  void parameter_event_cb(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
  {
    if (msg->node != "/waypoint_manager") {
      return;
    }
    for (const auto & changed : msg->changed_parameters) {
      if (changed.name == "buffer_wp") {
        fetch_mission_indices();
        break;
      }
    }
  }

  void update_waypoint_reached(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
  {
    waypoint_reached_ = msg->wp_seq;
    if (waypoint_reached_ == buffer_wp_) {
      stop_camera_trigger();
    }
  }

  void stop_camera_trigger()
  {
    // buffer_wp may be reached before any DigiCamCtrl ever started a timer.
    if (!timer_) {
      return;
    }
    timer_->cancel();
    timer_.reset();
    RCLCPP_INFO(get_logger(), "Camera trigger STOPPED");
    send_ack("Camera trigger STOPPED");
  }

  void statustext_callback(const mavros_msgs::msg::StatusText::SharedPtr msg)
  {
    if (msg->text.find("DigiCamCtrl") == std::string::npos) {
      return;
    }
    // Idempotent start: a second DigiCamCtrl must not stack another timer.
    if (timer_) {
      return;
    }
    timer_ = create_wall_timer(1s, [this]() { trigger_camera(); });
    RCLCPP_INFO(get_logger(), "Camera trigger STARTED");
    send_ack("Camera trigger STARTED");
  }

  void trigger_camera()
  {
    RCLCPP_INFO(get_logger(), "Triggering camera...");
    std_msgs::msg::Bool msg;
    msg.data = true;
    camera_trigger_pub_->publish(msg);
  }

  void send_ack(const std::string & text)
  {
    mavros_msgs::msg::StatusText msg;
    msg.severity = 6;  // INFO
    msg.text = text;
    status_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Status: %s", text.c_str());
  }

  int buffer_wp_{-1};
  int waypoint_reached_{0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr statustext_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr reached_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_trigger_pub_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr waypoint_param_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionCameraTrigger>());
  rclcpp::shutdown();
  return 0;
}
