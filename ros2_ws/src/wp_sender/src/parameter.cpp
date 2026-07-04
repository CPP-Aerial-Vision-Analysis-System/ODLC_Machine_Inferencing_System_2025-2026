// wp_sender: parameter manager (C++ port of wp_sender/parameter.py)
//
// Fetches parameters from waypoint_manager via the standard
// rcl_interfaces/GetParameters service and prints them. The demo main() reads
// the mission-index parameters, mirroring the Python entry point.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

using namespace std::chrono_literals;

class ParameterManager : public rclcpp::Node
{
public:
  ParameterManager()
  : Node("parameter_manager")
  {
    waypoint_client_ =
      create_client<rcl_interfaces::srv::GetParameters>("waypoint_manager/get_parameters");
    wait_for_services();
  }

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr waypoint_client() const
  {
    return waypoint_client_;
  }

  // Fetch parameters and log them as "name = value". Returns true on success.
  bool get_param(
    const rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr & client,
    const std::vector<std::string> & list_params)
  {
    auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    req->names = list_params;

    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Service call failed");
      return false;
    }

    auto result = future.get();
    if (result->values.size() != list_params.size()) {
      RCLCPP_ERROR(get_logger(), "No result or invalid response");
      return false;
    }

    for (size_t i = 0; i < list_params.size(); ++i) {
      RCLCPP_INFO(
        get_logger(), "%s = %s", list_params[i].c_str(), value_to_string(result->values[i]).c_str());
    }
    return true;
  }

private:
  static std::string value_to_string(const rcl_interfaces::msg::ParameterValue & val)
  {
    using rcl_interfaces::msg::ParameterType;
    switch (val.type) {
      case ParameterType::PARAMETER_BOOL:
        return val.bool_value ? "true" : "false";
      case ParameterType::PARAMETER_INTEGER:
        return std::to_string(val.integer_value);
      case ParameterType::PARAMETER_DOUBLE:
        return std::to_string(val.double_value);
      case ParameterType::PARAMETER_STRING:
        return val.string_value;
      default:
        return "<unsupported>";
    }
  }

  void wait_for_services()
  {
    while (!waypoint_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "waypoint_manager/get_parameters service not available, waiting...");
    }
  }

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr waypoint_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto param_manager = std::make_shared<ParameterManager>();
  param_manager->get_param(
    param_manager->waypoint_client(),
    {"num_waypoints", "takeoff_index", "rtl_index", "next_after_takeoff", "last_before_rtl"});
  rclcpp::shutdown();
  return 0;
}
