#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/image_result.hpp"
#include "interfaces/srv/add_waypoint.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/waypoint_set_current.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"

struct DetectionObject 
{
    std::string type;
    double confidence;
    double longitude;
    double latitude;
}

std::unordered_map<std::string, DetectionObject> detection = {
    {"person", {"person", 0.0, 0.0, 0.0}},
    {"tent", {"tent", 0.0, 0.0, 0.0}},
};

struct 
class Main : public rclcpp::Node
{
public: 
    Main() : Node("main")
    {
const auto detection_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    using std::placeholders::_1;
    using namespace std::chrono_literals;

    image_detection_sub_ = create_subscription<interfaces::msg::ImageResult>(
      "/image_detection", detection_qos,
      std::bind(&Main::image_result_cb, this, _1));
    waypoint_sub_ = create_subscription<mavros_msgs::msg::WaypointList>(
      "/mavros/mission/waypoints", 10,
      std::bind(&Main::waypoints_cb, this, _1));
    waypoint_reached_sub_ = create_subscription<mavros_msgs::msg::WaypointReached>(
      "/mavros/mission/reached", 1,
      std::bind(&Main::update_waypoint_reached, this, _1));
    parameter_event_sub_ = create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      std::bind(&Main::parameter_event_cb, this, _1));

    status_publisher_ = create_publisher<mavros_msgs::msg::StatusText>(
      "/mavros/statustext/send", 10);

    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    add_wp_client_ = create_client<interfaces::srv::AddWaypoint>("/addWaypoint");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    set_current_client_ = create_client<mavros_msgs::srv::WaypointSetCurrent>(
      "/mavros/mission/set_current");
    waypoint_param_client_ = create_client<rcl_interfaces::srv::GetParameters>(
      "/waypoint_manager/get_parameters");

    while (!set_mode_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "Set mode service not available, waiting ...");
    }
    while (!add_wp_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "Waiting for add waypoint service ...");
    }
    while (!waypoint_param_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "waypoint_manager/get_parameters service not available, waiting...");
    }
    while (!command_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "Waiting for command service ...");
    }

    declare_parameter<std::string>("camera_feed_path", "");
    camera_feed_path_ = resolve_camera_feed_path();

    detections_.emplace("person", DetectionObject{"person"});
    detections_.emplace("tent", DetectionObject{"tent"});

    fetch_mission_indices();
    }

private: 
  int last_before_rtl_{0};
  int next_after_takeoff_{0};
  int takeoff_index_{0};
  int rtl_index_{0};
  int num_waypoints_{0};
  int waypoint_reached_{0};
  int last_nav_before_rtl_{-1};
  int buffer_wp_{-1};
  std::size_t visit_index_{0};

  bool waiting_for_processing_{false};
  bool target_action_pending_{false};

  MissionPhase mission_phase_{MissionPhase::kSurvey};
  std::set<std::string> processed_image_names_;
  std::vector<mavros_msgs::msg::Waypoint> waypoints_;
  std::map<std::string, DetectionObject> detections_;
  std::vector<VisitTarget> visit_plan_;
  std::vector<int> pending_release_channels_;
  std::function<void()> after_servo_close_;
  std::string camera_feed_path_;

  rclcpp::TimerBase::SharedPtr processing_check_timer_;
  rclcpp::TimerBase::SharedPtr divert_timer_;
  rclcpp::TimerBase::SharedPtr resume_auto_timer_;
  rclcpp::TimerBase::SharedPtr target_loiter_timer_;
  rclcpp::TimerBase::SharedPtr servo_close_timer_;
  rclcpp::TimerBase::SharedPtr final_rtl_timer_;

  rclcpp::Subscription<interfaces::msg::ImageResult>::SharedPtr image_detection_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr waypoint_reached_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_publisher_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<interfaces::srv::AddWaypoint>::SharedPtr add_wp_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr set_current_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr waypoint_param_client_;

};

bool valid_detection(const std::string& object_type)
{
    auto it = detections_.find(object_type);

    if (it == detections_.end()) return false;
    if (it->second.confidence > 0) return true;
    
    return false;
}

 void fetch_mission_indices()
  {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

    request->names = {
      "num_waypoints",
      "takeoff_index",
      "rtl_index",
      "next_after_takeoff",
      "last_before_rtl"
    };

    waypoint_param_client_->async_send_request(
      request,
      [this, request](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future)
      {
        auto response = future.get();

        if (response->values.size() != request->names.size()) {
          RCLCPP_ERROR(get_logger(), "Parameter response size mismatch");
          return;
        }

        num_waypoints_ = static_cast<int>(response->values[0].integer_value);
        takeoff_index_ = static_cast<int>(response->values[1].integer_value);
        rtl_index_ = static_cast<int>(response->values[2].integer_value);
        next_after_takeoff_ = static_cast<int>(response->values[3].integer_value);
        last_before_rtl_ = static_cast<int>(response->values[4].integer_value);
      }
    );
  }


int main (int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Main>());
  rclcpp::shutdown();
  return 0;
}