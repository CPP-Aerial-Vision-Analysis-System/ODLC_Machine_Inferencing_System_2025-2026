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
#include <utility>
#include <vector>

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

namespace
{
constexpr double kAltitudeMeters = 16.8;
constexpr double kLoiterSeconds = 3.0;
constexpr double kServoOpenSeconds = 3.0;
constexpr int kPulleyOpenPwm = 1900;
constexpr int kPulleyClosePwm = 1400;

const std::vector<int> kPersonServoChannels{9, 11};
const std::vector<int> kTentServoChannels{13, 14};

struct DetectionObject
{
  std::string type;
  double confidence{0.0};
  double latitude{0.0};
  double longitude{0.0};
};

struct DetectionWaypoint
{
  double latitude;
  double longitude;
  double altitude;
  int index;
};

struct VisitTarget
{
  std::string type;
  double latitude;
  double longitude;
  double altitude;
  std::vector<int> servo_channels;
  int waypoint_index{-1};
};

enum class MissionPhase
{
  kSurvey,
  kProcessing,
  kVisiting,
  kDone,
};

std::chrono::milliseconds seconds_to_milliseconds(double seconds)
{
  return std::chrono::milliseconds(static_cast<int>(seconds * 1000.0));
}
}  // namespace

class MainController : public rclcpp::Node
{
public:
  MainController()
  : Node("main_controller")
  {
    const auto detection_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    using std::placeholders::_1;
    using namespace std::chrono_literals;

    image_detection_sub_ = create_subscription<interfaces::msg::ImageResult>(
      "/image_detection", detection_qos,
      std::bind(&MainController::image_result_cb, this, _1));
    waypoint_sub_ = create_subscription<mavros_msgs::msg::WaypointList>(
      "/mavros/mission/waypoints", 10,
      std::bind(&MainController::waypoints_cb, this, _1));
    waypoint_reached_sub_ = create_subscription<mavros_msgs::msg::WaypointReached>(
      "/mavros/mission/reached", 1,
      std::bind(&MainController::update_waypoint_reached, this, _1));
    parameter_event_sub_ = create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      std::bind(&MainController::parameter_event_cb, this, _1));

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

  void fetch_mission_indices()
  {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {
      "num_waypoints", "takeoff_index", "rtl_index", "next_after_takeoff", "last_before_rtl"};

    waypoint_param_client_->async_send_request(
      request,
      [this, request](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (response->values.size() != request->names.size()) {
            RCLCPP_ERROR(get_logger(), "Parameter response size mismatch");
            return;
          }

          num_waypoints_ = static_cast<int>(response->values[0].integer_value);
          takeoff_index_ = static_cast<int>(response->values[1].integer_value);
          rtl_index_ = static_cast<int>(response->values[2].integer_value);
          next_after_takeoff_ = static_cast<int>(response->values[3].integer_value);
          last_before_rtl_ = static_cast<int>(response->values[4].integer_value);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Failed to get waypoint manager parameters: %s", error.what());
        }
      });
  }

  void parameter_event_cb(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
  {
    if (msg->node != "/waypoint_manager") {
      return;
    }

    for (const auto & changed_parameter : msg->changed_parameters) {
      const auto & name = changed_parameter.name;
      if (
        name == "num_waypoints" || name == "takeoff_index" || name == "rtl_index" ||
        name == "next_after_takeoff" || name == "last_before_rtl")
      {
        fetch_mission_indices();
        return;
      }
    }
  }

  void waypoints_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg)
  {
    waypoints_ = msg->waypoints;
    update_last_nav_before_rtl();
  }

  void update_last_nav_before_rtl()
  {
    last_nav_before_rtl_ = -1;
    buffer_wp_ = -1;

    const int last_search_index = std::min(rtl_index_, static_cast<int>(waypoints_.size())) - 1;
    bool found_last_navigation_waypoint = false;

    for (int index = last_search_index; index >= 0; --index) {
      const auto command = waypoints_[static_cast<std::size_t>(index)].command;
      const bool is_navigation_command = command >= 16 && command <= 22;
      if (!is_navigation_command) {
        continue;
      }

      if (!found_last_navigation_waypoint) {
        last_nav_before_rtl_ = index;
        found_last_navigation_waypoint = true;
      } else {
        buffer_wp_ = index;
        break;
      }
    }

    if (last_nav_before_rtl_ >= 0) {
      RCLCPP_INFO(
        get_logger(), "Last nav WP before RTL: index %d (last_before_rtl=%d, rtl=%d)",
        last_nav_before_rtl_, last_before_rtl_, rtl_index_);
    }
    if (buffer_wp_ >= 0) {
      RCLCPP_INFO(get_logger(), "Buffer WP (GUIDED processing hold): index %d", buffer_wp_);
    }
  }

  void image_result_cb(const interfaces::msg::ImageResult::SharedPtr msg)
  {
    if (!msg->image_name.empty()) {
      processed_image_names_.insert(msg->image_name);
    }

    if (msg->detections.detections.empty()) {
      RCLCPP_INFO(get_logger(), "No objects detected.");
      send_ack("No objects detected.");
      return;
    }

    RCLCPP_INFO(get_logger(), "%zu object(s) detected!", msg->detections.detections.size());
    for (const auto & detection : msg->detections.detections) {
      for (const auto & result : detection.results) {
        std::string object_class;
        if (result.hypothesis.class_id == "0") {
          object_class = "person";
        } else if (result.hypothesis.class_id == "1") {
          object_class = "tent";
        } else {
          continue;
        }

        const double confidence = result.hypothesis.score;
        auto detection_it = detections_.find(object_class);
        if (detection_it == detections_.end() || confidence <= detection_it->second.confidence) {
          continue;
        }

        RCLCPP_INFO(
          get_logger(), "Updating %s: old_conf=%.2f, new_conf=%.2f", object_class.c_str(),
          detection_it->second.confidence, confidence);
        send_ack("Detected " + object_class);
        detection_it->second.confidence = confidence;
        detection_it->second.latitude = msg->latitude;
        detection_it->second.longitude = msg->longitude;
        RCLCPP_INFO(
          get_logger(), "Object at longitude: %.7f, latitude: %.7f",
          detection_it->second.longitude, detection_it->second.latitude);
      }
    }
  }

  void update_waypoint_reached(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
  {
    waypoint_reached_ = static_cast<int>(msg->wp_seq);
    const int trigger_waypoint = last_nav_before_rtl_ >= 0 ? last_nav_before_rtl_ : last_before_rtl_;

    if (mission_phase_ == MissionPhase::kSurvey && waypoint_reached_ == trigger_waypoint) {
      mission_phase_ = MissionPhase::kProcessing;
      send_ack("Reached last WP. Holding to finish image processing before RTL");
      RCLCPP_INFO(get_logger(), "Holding (GUIDED) at last WP; waiting for all images to be processed");
      change_mode("GUIDED");
      waiting_for_processing_ = true;
      if (!processing_check_timer_) {
        processing_check_timer_ = create_wall_timer(
          std::chrono::seconds(2), std::bind(&MainController::check_all_images_processed, this));
      }
      return;
    }

    if (
      mission_phase_ == MissionPhase::kVisiting && visit_index_ < visit_plan_.size() &&
      !target_action_pending_ && waypoint_reached_ == visit_plan_[visit_index_].waypoint_index)
    {
      loiter_and_drop(visit_plan_[visit_index_]);
    }
  }

  bool valid_detection(const std::string & object_type) const
  {
    const auto it = detections_.find(object_type);
    return it != detections_.end() && it->second.confidence > 0.0;
  }

  void check_all_images_processed()
  {
    if (mission_phase_ != MissionPhase::kProcessing) {
      cancel_timer(processing_check_timer_);
      return;
    }

    std::error_code error;
    if (!std::filesystem::exists(camera_feed_path_, error) || error) {
      RCLCPP_WARN(get_logger(), "Camera feed path not found: %s", camera_feed_path_.c_str());
      return;
    }

    std::set<std::string> image_files;
    for (const auto & entry : std::filesystem::directory_iterator(camera_feed_path_, error)) {
      if (error) {
        RCLCPP_ERROR(get_logger(), "Error reading camera feed path: %s", error.message().c_str());
        return;
      }
      if (!entry.is_regular_file()) {
        continue;
      }

      std::string extension = entry.path().extension().string();
      std::transform(
        extension.begin(), extension.end(), extension.begin(),
        [](unsigned char character) {return static_cast<char>(std::tolower(character));});
      if (extension == ".jpg" || extension == ".jpeg" || extension == ".png" || extension == ".bmp") {
        image_files.insert(entry.path().filename().string());
      }
    }

    std::size_t processed = 0;
    for (const auto & image_name : image_files) {
      if (processed_image_names_.count(image_name) != 0) {
        ++processed;
      }
    }
    const std::size_t total = image_files.size();
    const std::size_t remaining = total - processed;
    RCLCPP_INFO(get_logger(), "Processing check: %zu/%zu images done, %zu remaining", processed, total, remaining);

    if (remaining == 0) {
      send_ack("All " + std::to_string(total) + " images processed. Selecting targets.");
      cancel_timer(processing_check_timer_);
      waiting_for_processing_ = false;
      divert_to_targets();
    }
  }

  void divert_to_targets()
  {
    if (mission_phase_ != MissionPhase::kProcessing) {
      return;
    }

    visit_plan_.clear();
    add_visit_target_if_detected("tent", kTentServoChannels);
    add_visit_target_if_detected("person", kPersonServoChannels);

    if (visit_plan_.empty()) {
      send_ack("No valid targets found. Returning to launch.");
      RCLCPP_INFO(get_logger(), "No valid targets; commanding RTL");
      mission_phase_ = MissionPhase::kDone;
      change_mode("RTL");
      return;
    }

    const int base_index = last_before_rtl_;
    for (std::size_t index = 0; index < visit_plan_.size(); ++index) {
      visit_plan_[index].waypoint_index = base_index + 1 + static_cast<int>(index);
    }

    std::vector<DetectionWaypoint> insertion_list;
    insertion_list.reserve(visit_plan_.size());
    for (auto it = visit_plan_.rbegin(); it != visit_plan_.rend(); ++it) {
      insertion_list.push_back({it->latitude, it->longitude, it->altitude, base_index + 1});
    }
    send_waypoint_data(insertion_list);

    std::string names;
    for (const auto & target : visit_plan_) {
      if (!names.empty()) {
        names += " -> ";
      }
      names += target.type + "@" + std::to_string(target.waypoint_index);
    }
    send_ack("Targets: " + names + ". Diverting.");
    RCLCPP_INFO(get_logger(), "Visit plan: %s", names.c_str());

    mission_phase_ = MissionPhase::kVisiting;
    visit_index_ = 0;
    last_before_rtl_ = -1;
    divert_and_resume(visit_plan_.front().waypoint_index);
  }

  void add_visit_target_if_detected(const std::string & type, const std::vector<int> & servo_channels)
  {
    if (!valid_detection(type)) {
      return;
    }
    const auto & detection = detections_.at(type);
    visit_plan_.push_back({
      type, detection.latitude, detection.longitude, kAltitudeMeters, servo_channels, -1});
  }

  void send_waypoint_data(const std::vector<DetectionWaypoint> & waypoint_list)
  {
    RCLCPP_INFO(get_logger(), "Sending %zu waypoint(s)", waypoint_list.size());
    for (std::size_t index = 0; index < waypoint_list.size(); ++index) {
      const auto & waypoint = waypoint_list[index];
      auto request = std::make_shared<interfaces::srv::AddWaypoint::Request>();
      request->command = 16;
      request->latitude = waypoint.latitude;
      request->longitude = waypoint.longitude;
      request->altitude = waypoint.altitude;
      request->index = waypoint.index;
      request->channel = 0;
      request->pwm = 0;

      RCLCPP_INFO(
        get_logger(), "Waypoint %zu: lat=%.7f, lon=%.7f, alt=%.2f, index=%d", index + 1,
        waypoint.latitude, waypoint.longitude, waypoint.altitude, waypoint.index);
      add_wp_client_->async_send_request(
        request,
        [this, index](rclcpp::Client<interfaces::srv::AddWaypoint>::SharedFuture future) {
          try {
            if (!future.get()->success) {
              RCLCPP_ERROR(get_logger(), "Waypoint %zu insertion failed", index + 1);
            }
          } catch (const std::exception & error) {
            RCLCPP_ERROR(get_logger(), "Waypoint %zu request failed: %s", index + 1, error.what());
          }
        });
    }
  }

  void change_mode(const std::string & mode)
  {
    RCLCPP_INFO(get_logger(), "Setting mode to %s...", mode.c_str());
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    set_mode_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        try {
          if (future.get()->mode_sent) {
            RCLCPP_INFO(get_logger(), "Mode changed to %s", mode.c_str());
          } else {
            RCLCPP_ERROR(get_logger(), "Failed to change mode to %s", mode.c_str());
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "set_mode result error: %s", error.what());
        }
      });
  }

  void divert_and_resume(int target_sequence)
  {
    if (target_sequence < 0) {
      return;
    }
    cancel_timer(divert_timer_);
    divert_timer_ = create_wall_timer(
      std::chrono::seconds(4),
      [this, target_sequence]() {
        cancel_timer(divert_timer_);
        set_current_waypoint(target_sequence);
        resume_auto_timer_ = create_wall_timer(
          std::chrono::seconds(1),
          [this, target_sequence]() {
            cancel_timer(resume_auto_timer_);
            change_mode("AUTO");
            send_ack("Resumed AUTO -> WP " + std::to_string(target_sequence));
          });
      });
  }

  void set_current_waypoint(int sequence)
  {
    auto request = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
    request->wp_seq = static_cast<std::uint16_t>(sequence);
    set_current_client_->async_send_request(
      request,
      [this, sequence](rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedFuture future) {
        try {
          if (future.get()->success) {
            RCLCPP_INFO(get_logger(), "Set current waypoint -> %d", sequence);
            send_ack("Set current WP -> " + std::to_string(sequence));
          } else {
            RCLCPP_ERROR(get_logger(), "Failed to set current waypoint -> %d", sequence);
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "set_current result error: %s", error.what());
        }
      });
  }

  void loiter_and_drop(const VisitTarget & target)
  {
    target_action_pending_ = true;
    RCLCPP_INFO(
      get_logger(), "Arrived over %s (WP %d). Loitering %.0fs, then dropping.", target.type.c_str(),
      target.waypoint_index, kLoiterSeconds);
    send_ack("Over " + target.type + ": loiter " + std::to_string(static_cast<int>(kLoiterSeconds)) +
      "s then drop");
    change_mode("GUIDED");

    cancel_timer(target_loiter_timer_);
    target_loiter_timer_ = create_wall_timer(
      seconds_to_milliseconds(kLoiterSeconds),
      [this, channels = target.servo_channels]() {
        cancel_timer(target_loiter_timer_);
        actuate_servos(channels, [this]() {finish_target_drop();});
      });
  }

  void actuate_servos(const std::vector<int> & channels, std::function<void()> on_complete)
  {
    RCLCPP_INFO(
      get_logger(), "Opening %zu servos -> hold %.0fs -> close", channels.size(), kServoOpenSeconds);
    pending_release_channels_ = channels;
    after_servo_close_ = std::move(on_complete);
    for (const int channel : pending_release_channels_) {
      move_servo(channel, kPulleyOpenPwm);
    }

    cancel_timer(servo_close_timer_);
    servo_close_timer_ = create_wall_timer(
      seconds_to_milliseconds(kServoOpenSeconds),
      [this]() {
        cancel_timer(servo_close_timer_);
        for (const int channel : pending_release_channels_) {
          move_servo(channel, kPulleyClosePwm);
        }
        pending_release_channels_.clear();
        auto on_complete = std::move(after_servo_close_);
        after_servo_close_ = nullptr;
        if (on_complete) {
          on_complete();
        }
      });
  }

  void move_servo(int channel, int pwm)
  {
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->broadcast = false;
    request->command = 183;
    request->confirmation = 0;
    request->param1 = static_cast<float>(channel);
    request->param2 = static_cast<float>(pwm);
    request->param3 = 0.0F;
    request->param4 = 0.0F;
    request->param5 = 0.0F;
    request->param6 = 0.0F;
    request->param7 = 0.0F;

    command_client_->async_send_request(
      request,
      [this, channel, pwm](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
        try {
          if (future.get()->success) {
            RCLCPP_INFO(get_logger(), "[SERVO] Channel %d moved to %dus", channel, pwm);
            send_ack("Servo " + std::to_string(channel) + " -> " + std::to_string(pwm));
          } else {
            RCLCPP_WARN(get_logger(), "[SERVO] Failed to move channel %d", channel);
            send_ack("Servo " + std::to_string(channel) + " move FAILED");
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Servo result error: %s", error.what());
        }
      });
  }

  void finish_target_drop()
  {
    ++visit_index_;
    target_action_pending_ = false;
    if (visit_index_ < visit_plan_.size()) {
      change_mode("AUTO");
      return;
    }

    send_ack("All targets done. Waiting " +
      std::to_string(static_cast<int>(kLoiterSeconds)) + "s then RTL.");
    final_rtl_timer_ = create_wall_timer(
      seconds_to_milliseconds(kLoiterSeconds),
      [this]() {
        cancel_timer(final_rtl_timer_);
        mission_phase_ = MissionPhase::kDone;
        change_mode("RTL");
        send_ack("RTL");
      });
  }

  void send_ack(const std::string & text)
  {
    mavros_msgs::msg::StatusText message;
    message.severity = 6;
    message.text = text;
    status_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Status: %s", text.c_str());
  }

  std::string resolve_camera_feed_path()
  {
    const auto configured_path = get_parameter("camera_feed_path").as_string();
    if (!configured_path.empty()) {
      return configured_path;
    }

    std::error_code error;
    auto search_directory = std::filesystem::current_path(error);
    for (int depth = 0; !error && depth < 10; ++depth) {
      const auto workspace_source = search_directory / "src";
      if (std::filesystem::exists(workspace_source, error) &&
        std::filesystem::exists(search_directory / "install", error))
      {
        return (workspace_source / "video_cam" / "mapping_photos").string();
      }

      const auto nested_workspace = search_directory / "ros2_ws";
      if (std::filesystem::exists(nested_workspace / "src", error) &&
        std::filesystem::exists(nested_workspace / "install", error))
      {
        return (nested_workspace / "src" / "video_cam" / "mapping_photos").string();
      }

      const auto parent = search_directory.parent_path();
      if (parent == search_directory) {
        break;
      }
      search_directory = parent;
    }

    const auto source_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    if (std::filesystem::exists(source_path / "video_cam", error)) {
      return (source_path / "video_cam" / "mapping_photos").string();
    }
    return "/astra/ros2_ws/src/video_cam/mapping_photos";
  }

  void cancel_timer(rclcpp::TimerBase::SharedPtr & timer)
  {
    if (timer) {
      timer->cancel();
      timer.reset();
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainController>());
  rclcpp::shutdown();
  return 0;
}
