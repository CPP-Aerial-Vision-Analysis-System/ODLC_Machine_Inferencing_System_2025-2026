// waypoint_mavros: kill node (C++ port of waypoint_mavros/killnode.py)
//
// Listens for a shutdown command (a StatusText containing "systemid"), transfers
// the mapping photos to the camera_feed folder, then shuts the Jetson down.
//
// The Python original never wired its listener_callback to a subscription and
// called an undefined param_pull(); this port drops the dead param_pull() call
// and wires the clearly-intended StatusText subscription so the node actually
// reacts to shutdown commands.

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "mavros_msgs/msg/status_text.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class KillNode : public rclcpp::Node
{
public:
  KillNode()
  : Node("Kill_node"),
    src_folder_("/home/astra-dev/astra/ros2_ws/src/video_cam/mapping_photos"),
    dst_folder_("/home/astra-dev/astra/ros2_ws/src/video_cam/camera_feed")
  {
    message_sender_ =
      create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send", 10);

    status_sub_ = create_subscription<mavros_msgs::msg::StatusText>(
      "/mavros/statustext/recv", 10,
      [this](mavros_msgs::msg::StatusText::SharedPtr m) { listener_callback(m); });

    RCLCPP_INFO(get_logger(), "KillNode initialized and listening for shutdown commands.");
    send_back("Killnode is here");

    param_get_client_ =
      create_client<rcl_interfaces::srv::GetParameters>("/mavros/param/get_parameters");
    while (!param_get_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "param_get service not available, waiting...");
    }

    RCLCPP_INFO(get_logger(), "KillNode initialized and listening for shutdown commands.");
  }

private:
  void listener_callback(const mavros_msgs::msg::StatusText::SharedPtr msg)
  {
    std::string text = msg->text;
    std::transform(text.begin(), text.end(), text.begin(), ::tolower);
    if (text.find("systemid") != std::string::npos) {  // shutdown command received
      RCLCPP_WARN(get_logger(), "Jetson Shutdown Triggered. Shutting down...");
      send_back("Shutdown command received.");
      transfer_photos();
      RCLCPP_INFO(get_logger(), "Transfering completed");
      send_back("Transfering Completed, shutting down now");
      shutdown_jetson();
    }
  }

  void send_back(const std::string & text)
  {
    mavros_msgs::msg::StatusText msg;
    msg.severity = 6;  // NOTICE / INFO
    msg.text = text;
    message_sender_->publish(msg);
  }

  void transfer_photos()
  {
    std::error_code ec;
    fs::create_directories(dst_folder_, ec);
    if (!fs::exists(src_folder_)) {
      return;
    }
    for (const auto & entry : fs::directory_iterator(src_folder_, ec)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      std::string ext = entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
        fs::rename(entry.path(), fs::path(dst_folder_) / entry.path().filename(), ec);
      }
    }
  }

  void shutdown_jetson()
  {
    RCLCPP_INFO(get_logger(), "Executing Jetson shutdown command...");
    const std::string password = "UAV_Lab";
    std::string cmd = "echo " + password + " | sudo -S shutdown -h now";
    if (std::system(cmd.c_str()) != 0) {
      RCLCPP_ERROR(get_logger(), "Shutdown command failed");
    }
  }

  std::string src_folder_;
  std::string dst_folder_;

  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr message_sender_;
  rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr status_sub_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_get_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KillNode>());
  rclcpp::shutdown();
  return 0;
}
