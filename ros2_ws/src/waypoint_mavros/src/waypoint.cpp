// waypoint_mavros: waypoint manager (C++ port of waypoint_mavros/waypoint.py)
//
// Owns the vehicle mission via MAVROS: tracks the current WaypointList, exposes
// custom /addWaypoint, /delWaypoint and /updateMission services, and publishes
// the mission-index parameters (takeoff_index, rtl_index, ...) used by the main
// controller.
//
// Note on async: the Python version calls spin_until_future_complete from inside
// service callbacks (e.g. delete -> pull), which deadlocks a single-threaded
// executor. All MAVROS client calls here are fire-and-forget (async_send_request
// with a result callback) to stay deadlock-free while preserving behavior.

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "mavros_msgs/msg/command_code.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/waypoint_clear.hpp"
#include "mavros_msgs/srv/waypoint_pull.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"

#include "interfaces/srv/add_waypoint.hpp"
#include "interfaces/srv/del_waypoint.hpp"
#include "interfaces/srv/update_mission.hpp"

using namespace std::chrono_literals;

// One pending mission item to insert (NAV_WAYPOINT or DO_SET_SERVO).
struct WaypointInsert
{
  int command{16};
  int index{0};
  double lat{0.0};
  double lon{0.0};
  double alt{0.0};
  int channel{0};
  int pwm{0};
};

class WaypointManager : public rclcpp::Node
{
public:
  WaypointManager()
  : Node("waypoint_manager")
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    // Subscribers + publisher
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10, [this](mavros_msgs::msg::State::SharedPtr m) { state_callback(m); });
    reached_sub_ = create_subscription<mavros_msgs::msg::WaypointReached>(
      "/mavros/mission/reached", 10,
      [this](mavros_msgs::msg::WaypointReached::SharedPtr m) { waypoint_reached_cb(m); });
    waypoints_sub_ = create_subscription<mavros_msgs::msg::WaypointList>(
      "/mavros/mission/waypoints", 10,
      [this](mavros_msgs::msg::WaypointList::SharedPtr m) { waypoints_list(m); });
    status_publisher_ =
      create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send", 10);

    // MAVROS clients
    waypoint_pull_ = create_client<mavros_msgs::srv::WaypointPull>("/mavros/mission/pull");
    waypoint_push_ = create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");
    waypoint_clear_ = create_client<mavros_msgs::srv::WaypointClear>("/mavros/mission/clear");
    set_mode_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    // Custom services
    add_srv_ = create_service<interfaces::srv::AddWaypoint>(
      "/addWaypoint", std::bind(&WaypointManager::handle_wp_req, this, _1, _2));
    del_srv_ = create_service<interfaces::srv::DelWaypoint>(
      "/delWaypoint", std::bind(&WaypointManager::handle_wp_del_req, this, _1, _2));
    update_srv_ = create_service<interfaces::srv::UpdateMission>(
      "/updateMission", std::bind(&WaypointManager::handle_update_mission, this, _1, _2));

    // Node parameters
    declare_parameter<int>("num_waypoints", 0);
    declare_parameter<int>("takeoff_index", -1);
    declare_parameter<int>("rtl_index", -1);
    declare_parameter<int>("next_after_takeoff", -1);
    declare_parameter<int>("last_before_rtl", -1);
    declare_parameter<int>("buffer_wp", -1);
  }

  void run()
  {
    RCLCPP_INFO(get_logger(), "Waiting for connection to FCU...");
    while (rclcpp::ok() && !connected_) {
      rclcpp::spin_some(get_node_base_interface());
    }
    const std::string message = "Heartbeat established";
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    send_ack(message);
    pull_waypoints();
    rclcpp::spin(get_node_base_interface());
  }

private:
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    connected_ = msg->connected;
  }

  void waypoints_list(const mavros_msgs::msg::WaypointList::SharedPtr data)
  {
    waypoint_list_ = *data;
    for (size_t i = 0; i < waypoint_list_.waypoints.size(); ++i) {
      const auto & wp = waypoint_list_.waypoints[i];
      if (wp.command == mavros_msgs::msg::CommandCode::NAV_TAKEOFF) {
        takeoff_index_ = static_cast<int>(i);
        if (i + 1 < waypoint_list_.waypoints.size()) {
          next_after_takeoff_ = static_cast<int>(i) + 1;
        }
      }
      if (wp.command == mavros_msgs::msg::CommandCode::NAV_RETURN_TO_LAUNCH) {
        rtl_index_ = static_cast<int>(i);
        if (static_cast<int>(i) - 1 > 0) {
          last_before_rtl_ = static_cast<int>(i) - 1;
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Takeoff Index: %d, Next After Takeoff: %d, Last Before RTL: %d, RTL Index: %d",
      takeoff_index_, next_after_takeoff_, last_before_rtl_, rtl_index_);

    set_parameters({
      rclcpp::Parameter("num_waypoints", static_cast<int>(waypoint_list_.waypoints.size())),
      rclcpp::Parameter("takeoff_index", takeoff_index_),
      rclcpp::Parameter("next_after_takeoff", next_after_takeoff_),
      rclcpp::Parameter("rtl_index", rtl_index_),
      rclcpp::Parameter("last_before_rtl", last_before_rtl_),
      rclcpp::Parameter("buffer_wp", last_before_rtl_ - 1),
    });
  }

  void push_waypoints()
  {
    if (!waypoint_push_->service_is_ready()) {
      RCLCPP_INFO(get_logger(), "Waiting for waypoint push service...");
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    request->start_index = 0;
    request->waypoints = waypoint_list_.waypoints;
    waypoint_push_->async_send_request(
      request, [this](rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture future) {
        auto resp = future.get();
        if (resp && resp->success) {
          RCLCPP_INFO(get_logger(), "Waypoints pushed successfully");
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to push waypoints");
        }
      });
  }

  void pull_waypoints()
  {
    if (!waypoint_pull_->service_is_ready()) {
      RCLCPP_INFO(get_logger(), "Waiting for waypoint pull service...");
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::WaypointPull::Request>();
    waypoint_pull_->async_send_request(
      request, [this](rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedFuture future) {
        auto resp = future.get();
        if (resp && resp->success) {
          RCLCPP_INFO(get_logger(), "Successfully pulled %u waypoints from drone.", resp->wp_received);
        }
      });
    RCLCPP_INFO(get_logger(), "Waypoint pull request...");
  }

  bool insert_new_waypoint(const std::vector<WaypointInsert> & wp_list)
  {
    if (wp_list.empty()) {
      RCLCPP_WARN(get_logger(), "No waypoints to insert");
      return false;
    }

    auto original_waypoints = waypoint_list_.waypoints;

    for (const auto & wp : wp_list) {
      if (wp.index > static_cast<int>(waypoint_list_.waypoints.size())) {
        RCLCPP_ERROR(get_logger(), "Index %d is out of range", wp.index);
        waypoint_list_.waypoints = original_waypoints;
        return false;
      }

      mavros_msgs::msg::Waypoint new_waypoint;
      new_waypoint.is_current = false;
      new_waypoint.autocontinue = true;

      if (wp.command == 183) {  // MAV_CMD_DO_SET_SERVO
        new_waypoint.frame = 3;
        new_waypoint.command = 183;
        new_waypoint.param1 = static_cast<float>(wp.channel);
        new_waypoint.param2 = static_cast<float>(wp.pwm);
        new_waypoint.param3 = 0.0f;
        new_waypoint.param4 = 0.0f;
        new_waypoint.x_lat = 0.0;
        new_waypoint.y_long = 0.0;
        new_waypoint.z_alt = 0.0;
        RCLCPP_INFO(
          get_logger(), "Inserting DO_SET_SERVO at index %d: ch=%d, pwm=%d", wp.index, wp.channel,
          wp.pwm);
      } else {  // MAV_CMD_NAV_WAYPOINT
        new_waypoint.frame = 3;
        new_waypoint.command = 16;
        new_waypoint.param1 = 3.0f;  // Hold time
        new_waypoint.param2 = 0.0f;
        new_waypoint.param3 = 0.0f;
        new_waypoint.param4 = std::numeric_limits<float>::quiet_NaN();
        new_waypoint.x_lat = wp.lat;
        new_waypoint.y_long = wp.lon;
        new_waypoint.z_alt = wp.alt;
        RCLCPP_INFO(
          get_logger(), "Inserting NAV_WAYPOINT at index %d: lat=%f, lon=%f, alt=%f", wp.index,
          wp.lat, wp.lon, wp.alt);
      }

      auto & wps = waypoint_list_.waypoints;
      wps.insert(wps.begin() + std::min<size_t>(wp.index, wps.size()), new_waypoint);
    }

    push_waypoints();
    RCLCPP_INFO(get_logger(), "Successfully pushed %zu mission items", wp_list.size());
    return true;
  }

  void delete_waypoint(int index)
  {
    pull_waypoints();
    if (index >= 0 && index < static_cast<int>(waypoint_list_.waypoints.size())) {
      waypoint_list_.waypoints.erase(waypoint_list_.waypoints.begin() + index);
      RCLCPP_INFO(get_logger(), "Deleted waypoint at index %d.", index);
      push_waypoints();
      RCLCPP_INFO(get_logger(), "Waypoint deleted and pushed successfully.");
    } else {
      RCLCPP_INFO(get_logger(), "Index %d out of range. No waypoint deleted.", index);
    }
  }

  void waypoint_reached_cb(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
  {
    waypoint_reached_ = msg->wp_seq;
    RCLCPP_INFO(get_logger(), "Waypoint %u reached.", msg->wp_seq);

    if (waypoint_reached_ < static_cast<int>(waypoint_list_.waypoints.size())) {
      const auto & wp = waypoint_list_.waypoints[waypoint_reached_];
      if (static_cast<int>(wp.param1) > 0) {
        if (static_cast<int>(wp.command) == 16) {
          RCLCPP_INFO(get_logger(), "Object waypoint reached.");
          send_ack(
            "Object waypoint reached. Holding for " + std::to_string(static_cast<int>(wp.param1)) +
            " seconds.");
        } else {
          RCLCPP_INFO(get_logger(), "Loiter finished. Continuing to next waypoint.");
        }
      }
    }
  }

  void send_ack(const std::string & text)
  {
    mavros_msgs::msg::StatusText msg;
    msg.severity = 6;  // INFO
    msg.text = text;
    status_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Status: %s", text.c_str());
  }

  void handle_wp_req(
    const std::shared_ptr<interfaces::srv::AddWaypoint::Request> request,
    std::shared_ptr<interfaces::srv::AddWaypoint::Response> response)
  {
    const int command = request->command != 0 ? request->command : 16;

    WaypointInsert wp;
    wp.command = command;
    wp.index = request->index;
    if (command == 183) {
      RCLCPP_INFO(
        get_logger(), "Received DO_SET_SERVO request: ch=%d, pwm=%d, index=%d", request->channel,
        request->pwm, request->index);
      wp.channel = request->channel;
      wp.pwm = request->pwm;
    } else {
      RCLCPP_INFO(
        get_logger(), "Received NAV_WAYPOINT request: lat=%f, lon=%f, alt=%f, index=%d",
        request->latitude, request->longitude, request->altitude, request->index);
      wp.lat = request->latitude;
      wp.lon = request->longitude;
      wp.alt = request->altitude;
    }

    response->success = insert_new_waypoint({wp});
  }

  void handle_wp_del_req(
    const std::shared_ptr<interfaces::srv::DelWaypoint::Request> request,
    std::shared_ptr<interfaces::srv::DelWaypoint::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Received DelWaypoint request: index=%d", request->index);
    delete_waypoint(request->index);
    response->success = true;
  }

  void handle_update_mission(
    const std::shared_ptr<interfaces::srv::UpdateMission::Request> /*request*/,
    std::shared_ptr<interfaces::srv::UpdateMission::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Received UpdateMission request");
    response->success = true;  // nothing coded yet
  }

  // State
  bool connected_{false};
  int waypoint_reached_{0};
  mavros_msgs::msg::WaypointList waypoint_list_;
  int takeoff_index_{-1};
  int next_after_takeoff_{-1};
  int rtl_index_{-1};
  int last_before_rtl_{-1};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr reached_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoints_sub_;
  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_publisher_;

  rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedPtr waypoint_pull_;
  rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr waypoint_push_;
  rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr waypoint_clear_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_;

  rclcpp::Service<interfaces::srv::AddWaypoint>::SharedPtr add_srv_;
  rclcpp::Service<interfaces::srv::DelWaypoint>::SharedPtr del_srv_;
  rclcpp::Service<interfaces::srv::UpdateMission>::SharedPtr update_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto manager = std::make_shared<WaypointManager>();
  manager->run();
  rclcpp::shutdown();
  return 0;
}
