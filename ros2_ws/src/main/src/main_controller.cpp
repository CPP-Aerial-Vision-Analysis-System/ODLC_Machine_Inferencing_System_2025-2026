// main: mission controller (C++ port of main/src/main_controller.py)
//
// Orchestrates the ODLC mission: watches object-detection results while the
// survey mission runs, holds in GUIDED at the last waypoint until all captured
// images are processed, then diverts to the highest-confidence tent and person,
// loiters, actuates the release servos, and finally returns to launch.
//
// Mission-index parameters are read from waypoint_manager via GetParameters
// (inlined here, replacing the Python wp_sender.ParameterManager import). All
// service-client calls made from callbacks are fire-and-forget to keep the
// single-threaded executor deadlock-free.

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
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

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace
{
constexpr double ALT = 16.8;  // meters (~55 ft)

// Each target releases via TWO servos.
const std::vector<int> PERSON_SERVO_CHANNELS = {9, 11};
const std::vector<int> TENT_SERVO_CHANNELS = {13, 14};

constexpr int PULLEY_OPEN = 1900;   // released / open
constexpr int PULLEY_CLOSE = 1400;  // closed
constexpr double SERVO_OPEN_SECONDS = 3.0;
}  // namespace

struct DetectionObject
{
  std::string type;
  double confidence{0.0};
  double lat{0.0};
  double lon{0.0};
};

struct VisitTarget
{
  std::string type;
  double lat{0.0};
  double lon{0.0};
  double alt{ALT};
  std::vector<int> servos;
  int wp{-1};
};

class MainController : public rclcpp::Node
{
public:
  MainController()
  : Node("main_controller")
  {
    // Subscribers
    const auto detection_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    image_sub_ = create_subscription<interfaces::msg::ImageResult>(
      "/image_detection", detection_qos,
      [this](interfaces::msg::ImageResult::SharedPtr m) { image_result_cb(m); });
    waypoints_sub_ = create_subscription<mavros_msgs::msg::WaypointList>(
      "/mavros/mission/waypoints", 10,
      [this](mavros_msgs::msg::WaypointList::SharedPtr m) { waypoints_cb(m); });
    reached_sub_ = create_subscription<mavros_msgs::msg::WaypointReached>(
      "/mavros/mission/reached", 1,
      [this](mavros_msgs::msg::WaypointReached::SharedPtr m) { update_waypoint_reached(m); });
    parameter_event_sub_ = create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      [this](rcl_interfaces::msg::ParameterEvent::SharedPtr m) { parameter_event_cb(m); });

    // Publisher
    status_publisher_ =
      create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send", 10);

    // Clients
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    while (!set_mode_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "Set mode service not available, waiting ...");
    }
    add_wp_client_ = create_client<interfaces::srv::AddWaypoint>("/addWaypoint");
    while (!add_wp_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "Waiting for add waypoint service ...");
    }
    command_client_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    set_current_client_ =
      create_client<mavros_msgs::srv::WaypointSetCurrent>("/mavros/mission/set_current");
    waypoint_param_client_ =
      create_client<rcl_interfaces::srv::GetParameters>("waypoint_manager/get_parameters");

    // Detection state
    detections_.emplace("person", DetectionObject{"person", 0.0, 0.0, 0.0});
    detections_.emplace("tent", DetectionObject{"tent", 0.0, 0.0, 0.0});

    declare_parameter<std::string>("camera_feed_path", "");
    camera_feed_path_ = resolve_camera_feed_path();

    fetch_mission_indices();

    while (!command_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "Waiting for command service ...");
    }
  }

private:
  // --- Mission-index parameters ---------------------------------------------
  void fetch_mission_indices()
  {
    if (!waypoint_param_client_->service_is_ready()) {
      // Not fatal: parameter_events will trigger a re-fetch once available.
      return;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {
      "num_waypoints", "takeoff_index", "rtl_index", "next_after_takeoff", "last_before_rtl"};

    waypoint_param_client_->async_send_request(
      request,
      [this, request](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
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
      });
  }

  void parameter_event_cb(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
  {
    if (msg->node != "/waypoint_manager") {
      return;
    }
    static const std::set<std::string> kWatched = {
      "num_waypoints", "takeoff_index", "rtl_index", "next_after_takeoff", "last_before_rtl"};
    for (const auto & changed : msg->changed_parameters) {
      if (kWatched.count(changed.name)) {
        fetch_mission_indices();
        break;
      }
    }
  }

  // --- Waypoint tracking -----------------------------------------------------
  void update_waypoint_reached(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
  {
    waypoint_reached_ = msg->wp_seq;

    const int trigger_wp = last_nav_before_rtl_ >= 0 ? last_nav_before_rtl_ : last_before_rtl_;

    if (mission_phase_ == "survey" && waypoint_reached_ == trigger_wp) {
      mission_phase_ = "processing";
      send_ack("Reached last WP. Holding to finish image processing before RTL");
      RCLCPP_INFO(
        get_logger(), "Holding (GUIDED) at last WP; waiting for all images to be processed");
      change_mode("GUIDED");
      waiting_for_processing_ = true;
      if (!processing_check_timer_) {
        processing_check_timer_ =
          create_wall_timer(2s, [this]() { check_all_images_processed(); });
      }
      return;
    }

    if (mission_phase_ == "visiting" && visit_idx_ < static_cast<int>(visit_plan_.size())) {
      const auto & target = visit_plan_[visit_idx_];
      if (waypoint_reached_ == target.wp) {
        loiter_and_drop(target);
      }
    }
  }

  void divert_to_targets()
  {
    if (mission_phase_ != "processing") {
      return;
    }

    const std::vector<std::pair<std::string, std::vector<int>>> order = {
      {"tent", TENT_SERVO_CHANNELS},
      {"person", PERSON_SERVO_CHANNELS},
    };
    visit_plan_.clear();
    for (const auto & [obj_type, servos] : order) {
      if (valid_detection(obj_type)) {
        const auto & d = detections_.at(obj_type);
        VisitTarget t;
        t.type = obj_type;
        t.lat = d.lat;
        t.lon = d.lon;
        t.alt = ALT;
        t.servos = servos;
        t.wp = -1;
        visit_plan_.push_back(t);
      }
    }

    if (visit_plan_.empty()) {
      send_ack("No valid targets found. Returning to launch.");
      RCLCPP_INFO(get_logger(), "No valid targets; commanding RTL");
      mission_phase_ = "done";
      change_mode("RTL");
      return;
    }

    const int base = last_before_rtl_;
    for (size_t i = 0; i < visit_plan_.size(); ++i) {
      visit_plan_[i].wp = base + 1 + static_cast<int>(i);
    }

    // waypoint_manager inserts at the index (pushing later items down); insert
    // in reverse so visit_plan_[0] lands at base+1.
    std::vector<AddWp> inject;
    for (auto it = visit_plan_.rbegin(); it != visit_plan_.rend(); ++it) {
      inject.push_back(AddWp{it->lat, it->lon, it->alt, base + 1});
    }
    send_waypoint_data(inject);

    std::string names;
    for (size_t i = 0; i < visit_plan_.size(); ++i) {
      if (i) {
        names += " -> ";
      }
      names += visit_plan_[i].type + "@" + std::to_string(visit_plan_[i].wp);
    }
    send_ack("Targets: " + names + ". Diverting.");
    RCLCPP_INFO(get_logger(), "Visit plan: %s", names.c_str());

    mission_phase_ = "visiting";
    visit_idx_ = 0;
    last_before_rtl_ = -1;
    divert_and_resume(visit_plan_[0].wp);
  }

  void loiter_and_drop(const VisitTarget & target)
  {
    RCLCPP_INFO(
      get_logger(), "Arrived over %s (WP %d). Loitering %.0fs, then dropping.",
      target.type.c_str(), target.wp, LOITER_SECONDS_);
    send_ack("Over " + target.type + ": loiter then drop");
    change_mode("GUIDED");
    std::this_thread::sleep_for(std::chrono::duration<double>(LOITER_SECONDS_));
    actuate_servos(target.servos);
    ++visit_idx_;

    if (visit_idx_ < static_cast<int>(visit_plan_.size())) {
      change_mode("AUTO");
    } else {
      send_ack("All targets done. Waiting then RTL.");
      std::this_thread::sleep_for(std::chrono::duration<double>(LOITER_SECONDS_));
      mission_phase_ = "done";
      change_mode("RTL");
      send_ack("RTL");
    }
  }

  bool valid_detection(const std::string & type)
  {
    auto it = detections_.find(type);
    return it != detections_.end() && it->second.confidence > 0;
  }

  void waypoints_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg)
  {
    waypoints_ = msg->waypoints;
    update_last_nav_before_rtl();
  }

  void update_last_nav_before_rtl()
  {
    // NAV_WAYPOINT, NAV_LOITER_*, NAV_RETURN_TO_LAUNCH, NAV_TAKEOFF
    static const std::set<int> NAV_COMMANDS = {16, 17, 18, 19, 20, 21, 22};
    last_nav_before_rtl_ = -1;
    buffer_wp_ = -1;
    if (rtl_index_ > 0 && !waypoints_.empty()) {
      bool found_last = false;
      const int upper = std::min<int>(rtl_index_, static_cast<int>(waypoints_.size()));
      for (int i = upper - 1; i >= 0; --i) {
        if (NAV_COMMANDS.count(waypoints_[i].command)) {
          if (!found_last) {
            last_nav_before_rtl_ = i;
            found_last = true;
          } else {
            buffer_wp_ = i;
            break;
          }
        }
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

  std::string resolve_camera_feed_path()
  {
    std::string param = get_parameter("camera_feed_path").as_string();
    if (!param.empty()) {
      return param;
    }
    // Walk up from the current directory looking for a ros2_ws (has install+src).
    fs::path search = fs::current_path();
    fs::path ros2_ws;
    for (int i = 0; i < 10; ++i) {
      if (fs::exists(search / "install") && fs::exists(search / "src")) {
        ros2_ws = search;
        break;
      }
      if (search.parent_path() == search) {
        break;
      }
      search = search.parent_path();
    }
    fs::path base = !ros2_ws.empty() ? ros2_ws / "src" : fs::path("/astra/ros2_ws/src");
    return (base / "video_cam" / "mapping_photos").string();
  }

  void check_all_images_processed()
  {
    if (mission_phase_ != "processing") {
      if (processing_check_timer_) {
        processing_check_timer_->cancel();
        processing_check_timer_.reset();
      }
      return;
    }
    std::error_code ec;
    if (!fs::exists(camera_feed_path_)) {
      RCLCPP_WARN(get_logger(), "Camera feed path not found: %s", camera_feed_path_.c_str());
      return;
    }

    std::set<std::string> image_files;
    for (const auto & entry : fs::directory_iterator(camera_feed_path_, ec)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      std::string ext = entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
        image_files.insert(entry.path().filename().string());
      }
    }
    const int total = static_cast<int>(image_files.size());
    int processed = 0;
    for (const auto & name : processed_image_names_) {
      if (image_files.count(name)) {
        ++processed;
      }
    }
    const int remaining = total - processed;
    RCLCPP_INFO(
      get_logger(), "Processing check: %d/%d images done, %d remaining", processed, total,
      remaining);

    if (remaining <= 0) {
      send_ack("All " + std::to_string(total) + " images processed. Selecting targets.");
      if (processing_check_timer_) {
        processing_check_timer_->cancel();
        processing_check_timer_.reset();
      }
      waiting_for_processing_ = false;
      divert_to_targets();
    }
  }

  // --- Detection results -----------------------------------------------------
  void image_result_cb(const interfaces::msg::ImageResult::SharedPtr msg)
  {
    if (!msg->image_name.empty()) {
      processed_image_names_.insert(msg->image_name);
    }

    if (!msg->detections.detections.empty()) {
      RCLCPP_INFO(
        get_logger(), "%zu object(s) detected!", msg->detections.detections.size());
      for (const auto & detection : msg->detections.detections) {
        for (const auto & result : detection.results) {
          const std::string obj_id = result.hypothesis.class_id;
          std::string obj_class;
          if (obj_id == "0") {
            obj_class = "person";
          } else if (obj_id == "1") {
            obj_class = "tent";
          } else {
            continue;  // not an actionable mission target
          }
          const double obj_conf = result.hypothesis.score;

          auto it = detections_.find(obj_class);
          if (it != detections_.end() && obj_conf > it->second.confidence) {
            RCLCPP_INFO(
              get_logger(), "Updating %s: old_conf=%.2f, new_conf=%.2f", obj_class.c_str(),
              it->second.confidence, obj_conf);
            send_ack("Detected " + obj_class);
            it->second.confidence = obj_conf;
            it->second.lat = msg->latitude;
            it->second.lon = msg->longitude;
            RCLCPP_INFO(
              get_logger(), "Obj at long: %f, lat: %f", it->second.lon, it->second.lat);
          }
        }
      }
    } else {
      RCLCPP_INFO(get_logger(), "No objects detected.");
      send_ack("No objects detected.");
    }
  }

  // --- Flight-mode + servo actuation ----------------------------------------
  void change_mode(const std::string & mode)
  {
    RCLCPP_INFO(get_logger(), "Setting mode to %s...", mode.c_str());
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    set_mode_client_->async_send_request(
      req, [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        auto response = future.get();
        if (response && response->mode_sent) {
          RCLCPP_INFO(get_logger(), "Mode changed to %s", mode.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to change mode to %s", mode.c_str());
        }
      });
  }

  struct AddWp
  {
    double lat;
    double lon;
    double alt;
    int index;
  };

  void send_waypoint_data(const std::vector<AddWp> & wp_list)
  {
    RCLCPP_INFO(get_logger(), "Sending %zu waypoints", wp_list.size());
    for (size_t i = 0; i < wp_list.size(); ++i) {
      const auto & wp = wp_list[i];
      auto req = std::make_shared<interfaces::srv::AddWaypoint::Request>();
      req->command = 16;  // NAV_WAYPOINT
      req->latitude = wp.lat;
      req->longitude = wp.lon;
      req->altitude = wp.alt;
      req->index = wp.index;
      req->channel = 0;
      req->pwm = 0;
      RCLCPP_INFO(
        get_logger(), "Waypoint %zu: lat=%f, lon=%f, alt=%f, index=%d", i + 1, wp.lat, wp.lon,
        wp.alt, wp.index);
      add_wp_client_->async_send_request(req);
    }
  }

  void set_current_wp(int seq)
  {
    auto req = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
    req->wp_seq = seq;
    set_current_client_->async_send_request(req);
    RCLCPP_INFO(get_logger(), "Set current waypoint -> %d", seq);
    send_ack("Set current WP -> " + std::to_string(seq));
  }

  void divert_and_resume(int target_seq)
  {
    if (target_seq < 0) {
      return;
    }
    std::this_thread::sleep_for(4s);
    set_current_wp(target_seq);
    std::this_thread::sleep_for(1s);
    change_mode("AUTO");
    send_ack("Resumed AUTO -> WP " + std::to_string(target_seq));
  }

  void actuate_servos(const std::vector<int> & channels)
  {
    RCLCPP_INFO(get_logger(), "Opening servos -> hold %.0fs -> close", SERVO_OPEN_SECONDS);
    for (int ch : channels) {
      move_servo(ch, PULLEY_OPEN);
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(SERVO_OPEN_SECONDS));
    for (int ch : channels) {
      move_servo(ch, PULLEY_CLOSE);
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

    command_client_->async_send_request(
      request,
      [this, channel, pwm](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
        auto response = future.get();
        if (response && response->success) {
          RCLCPP_INFO(get_logger(), "[SERVO] Channel %d moved to %dus", channel, pwm);
          send_ack("Servo " + std::to_string(channel) + " -> " + std::to_string(pwm));
        } else {
          RCLCPP_WARN(get_logger(), "[SERVO] Failed to move channel %d", channel);
          send_ack("Servo " + std::to_string(channel) + " move FAILED");
        }
      });
  }

  void send_ack(const std::string & text)
  {
    mavros_msgs::msg::StatusText msg;
    msg.severity = 6;  // INFO
    msg.text = text;
    status_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Status: %s", text.c_str());
  }

  // --- State ----------------------------------------------------------------
  int last_before_rtl_{0};
  int next_after_takeoff_{0};
  int takeoff_index_{0};
  int rtl_index_{0};
  int num_waypoints_{0};
  int waypoint_reached_{0};
  int human_wp_{-1};
  int tent_wp_{-1};
  int last_nav_before_rtl_{-1};
  int buffer_wp_{-1};
  int visit_idx_{0};

  bool waiting_for_processing_{false};
  double LOITER_SECONDS_{3.0};

  std::string mission_phase_{"survey"};
  std::set<std::string> processed_image_names_;
  std::vector<mavros_msgs::msg::Waypoint> waypoints_;
  std::map<std::string, DetectionObject> detections_;
  std::vector<VisitTarget> visit_plan_;
  std::string camera_feed_path_;

  rclcpp::TimerBase::SharedPtr processing_check_timer_;

  rclcpp::Subscription<interfaces::msg::ImageResult>::SharedPtr image_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr reached_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_publisher_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<interfaces::srv::AddWaypoint>::SharedPtr add_wp_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr set_current_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr waypoint_param_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainController>());
  rclcpp::shutdown();
  return 0;
}
