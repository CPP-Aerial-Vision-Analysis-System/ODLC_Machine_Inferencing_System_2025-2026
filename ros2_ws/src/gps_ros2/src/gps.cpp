// gps_ros2: GPS/MAVROS service node (C++ port of gps_ros2/gps.py)
//
// Keeps the same node name ('gps_mavros_service_node'), the same topics, and
// the same service (/get_drone_data). Subscribes to MAVROS state, GPS fix and
// compass heading; serves the latest values via the GetGPSData service; and
// sets the MAVROS stream rate once at startup.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/stream_rate.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"

#include "interfaces/srv/get_gps_data.hpp"

using namespace std::chrono_literals;

class GPSMavrosServiceNode : public rclcpp::Node
{
public:
  GPSMavrosServiceNode()
  : Node("gps_mavros_service_node")
  {
    // Subscribers
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](mavros_msgs::msg::State::SharedPtr msg) { state_cb(msg); });

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) { gps_cb(msg); });

    yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64::SharedPtr msg) { pose_callback(msg); });

    // Service server
    service_ = create_service<interfaces::srv::GetGPSData>(
      "/get_drone_data",
      [this](
        const std::shared_ptr<interfaces::srv::GetGPSData::Request> req,
        std::shared_ptr<interfaces::srv::GetGPSData::Response> res) {
        handle_drone_data_request(req, res);
      });
    RCLCPP_INFO(get_logger(), "Drone Data Service Ready on /get_drone_data");

    // Service client for setting the MAVROS stream rate
    stream_rate_cli_ = create_client<mavros_msgs::srv::StreamRate>("/mavros/set_stream_rate");
    while (!stream_rate_cli_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Shutting down while waiting for /mavros/set_stream_rate");
        break;
      }
      RCLCPP_WARN(get_logger(), "Waiting for /mavros/set_stream_rate ...");
    }

    // Heartbeat timer (1 Hz), logs only on connection-state changes.
    heartbeat_timer_ = create_wall_timer(1s, [this]() { heartbeat_tick(); });

    // Set the stream rate once at startup (fire-and-forget).
    set_stream_rate(0, 1, true);
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    connected_ = msg->connected;
  }

  void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    latest_gps_ = msg;
  }

  void pose_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_yaw_ = msg->data;
    have_yaw_ = true;
  }

  void heartbeat_tick()
  {
    if (!last_connected_valid_ || connected_ != last_connected_) {
      if (connected_) {
        RCLCPP_INFO(get_logger(), "Heartbeat: Connected to Pixhawk");
      } else {
        RCLCPP_WARN(get_logger(), "Heartbeat: Disconnected from Pixhawk");
      }
      last_connected_ = connected_;
      last_connected_valid_ = true;
    }
  }

  void set_stream_rate(uint16_t stream_id, uint16_t message_rate, bool on_off)
  {
    if (!stream_rate_cli_->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(), "Service /mavros/set_stream_rate not ready (skipping initial call).");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::StreamRate::Request>();
    req->stream_id = stream_id;
    req->message_rate = message_rate;
    req->on_off = on_off;

    // Fire-and-forget: the constructor runs before spinning, so we cannot spin
    // here to block on the result. StreamRate has an empty response anyway.
    stream_rate_cli_->async_send_request(
      req,
      [this, stream_id, message_rate](
        rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedFuture future) {
        (void)future;
        RCLCPP_INFO(
          get_logger(), "Stream rate set: ID=%u, Rate=%uHz", stream_id, message_rate);
      });
  }

  void handle_drone_data_request(
    const std::shared_ptr<interfaces::srv::GetGPSData::Request> /*request*/,
    std::shared_ptr<interfaces::srv::GetGPSData::Response> response)
  {
    if (latest_gps_ != nullptr && have_yaw_) {
      response->latitude = latest_gps_->latitude;
      response->longitude = latest_gps_->longitude;
      response->altitude = latest_gps_->altitude;
      response->yaw = latest_yaw_;
    } else {
      RCLCPP_WARN(get_logger(), "No GPS or yaw data received yet!");
      response->latitude = 0.0;
      response->longitude = 0.0;
      response->altitude = 0.0;
      response->yaw = 0.0;
    }
  }

  // State
  bool connected_{false};
  bool last_connected_{false};
  bool last_connected_valid_{false};
  sensor_msgs::msg::NavSatFix::SharedPtr latest_gps_;
  double latest_yaw_{0.0};
  bool have_yaw_{false};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
  rclcpp::Service<interfaces::srv::GetGPSData>::SharedPtr service_;
  rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedPtr stream_rate_cli_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSMavrosServiceNode>());
  rclcpp::shutdown();
  return 0;
}
