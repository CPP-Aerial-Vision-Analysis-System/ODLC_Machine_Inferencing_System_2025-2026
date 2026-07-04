// wp_sender: waypoint client (C++ port of wp_sender/waypoint_sender.py)
//
// Thin client around the custom /addWaypoint and /delWaypoint services exposed
// by waypoint_manager. The demo main() adds a NAV waypoint then deletes it,
// mirroring the Python entry point.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/srv/add_waypoint.hpp"
#include "interfaces/srv/del_waypoint.hpp"

using namespace std::chrono_literals;

class WaypointClient : public rclcpp::Node
{
public:
  WaypointClient()
  : Node("waypoint_client")
  {
    add_wp_client_ = create_client<interfaces::srv::AddWaypoint>("/addWaypoint");
    del_wp_client_ = create_client<interfaces::srv::DelWaypoint>("/delWaypoint");
    wait_for_services();
  }

  // Insert a NAV_WAYPOINT at the given index. Returns success flag (or false).
  bool send_add_wp_request(double lon, double lat, double alt, int index)
  {
    RCLCPP_INFO(get_logger(), "AddWayPoint function called");
    auto request = std::make_shared<interfaces::srv::AddWaypoint::Request>();
    request->command = 16;  // NAV_WAYPOINT
    request->latitude = lat;
    request->longitude = lon;
    request->altitude = alt;
    request->index = index;
    request->channel = 0;
    request->pwm = 0;

    auto future = add_wp_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "AddWayPoint service call failed");
      return false;
    }
    return future.get()->success;
  }

  // Insert a DO_SET_SERVO mission item at the given index.
  bool send_servo_wp_request(int channel, int pwm, int index)
  {
    RCLCPP_INFO(
      get_logger(), "Adding DO_SET_SERVO: ch=%d, pwm=%d, index=%d", channel, pwm, index);
    auto request = std::make_shared<interfaces::srv::AddWaypoint::Request>();
    request->command = 183;  // MAV_CMD_DO_SET_SERVO
    request->latitude = 0.0;
    request->longitude = 0.0;
    request->altitude = 0.0;
    request->index = index;
    request->channel = channel;
    request->pwm = pwm;

    auto future = add_wp_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "ServoWP service call failed");
      return false;
    }
    return future.get()->success;
  }

  bool send_del_wp_request(int index)
  {
    RCLCPP_INFO(get_logger(), "DelWayPoint function called");
    auto request = std::make_shared<interfaces::srv::DelWaypoint::Request>();
    request->index = index;

    auto future = del_wp_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "DelWayPoint service call failed");
      return false;
    }
    return future.get()->success;
  }

private:
  void wait_for_services()
  {
    while (!add_wp_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "/addWaypoint service not available, waiting...");
    }
    while (!del_wp_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "/delWaypoint service not available, waiting...");
    }
  }

  rclcpp::Client<interfaces::srv::AddWaypoint>::SharedPtr add_wp_client_;
  rclcpp::Client<interfaces::srv::DelWaypoint>::SharedPtr del_wp_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto wp_client = std::make_shared<WaypointClient>();

  bool r1 = wp_client->send_add_wp_request(30.2, 10.5, 4.2, 0);
  RCLCPP_INFO(wp_client->get_logger(), "AddWaypoint success=%s", r1 ? "true" : "false");

  bool r2 = wp_client->send_del_wp_request(0);
  RCLCPP_INFO(wp_client->get_logger(), "DelWaypoint success=%s", r2 ? "true" : "false");

  rclcpp::shutdown();
  return 0;
}
