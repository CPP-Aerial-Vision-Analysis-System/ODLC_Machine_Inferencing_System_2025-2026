#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/image_result.hpp"
#include "interfaces/srv/add_waypoint.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"


using std::string;
using std::vector;
using std::map;
using std::set;
using std::bind;
using std::make_shared;
using std::placeholders::_1;

constexpr double ALT = 16.8;

struct DetectionObject
{
    string type;
    double confidence;
    double lat;
    double lon;
};

struct DetectionWaypoint
{
    double lat;
    double lon;
    double alt;
    int index;
};


class MainController : public rclcpp::Node
{
public:
    MainController() : Node("main_controller")
    {
        auto detection_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        using namespace std::chrono_literals;
        
        //subscribers
        image_detection_sub=this->create_subscription<interfaces::msg::ImageResult>("/image_detection", detection_qos,  std::bind(&MainController::image_result_cb, this,_1));
        waypoint_sub=this->create_subscription<mavros_msgs::msg::WaypointList>("/mavros/mission/waypoints", 10,  std::bind(&MainController::waypoints_cb, this,_1));
        waypoint_reached_sub=this->create_subscription<mavros_msgs::msg::WaypointReached>("/mavros/mission/reached", 10,  std::bind(&MainController::update_waypoint_reached, this,_1));
        parameters_event_sub=this->create_subscription<rcl_interfaces::msg::ParameterEvent>("/parameter_events", 10,  std::bind(&MainController::parameter_event_cb, this, _1));


        // Publishers
        status_publisher_ =this->create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send",10);

        // Clients
        set_mode_client_ =this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        while (!set_mode_client_->wait_for_service(1s)) RCLCPP_INFO(this->get_logger(), "Set mode service not available, waiting ...");
        add_wp_client_ =this->create_client<interfaces::srv::AddWaypoint>("/addWaypoint");
        while (!add_wp_client_->wait_for_service(1s)) RCLCPP_INFO(this->get_logger(), "Waiting for add waypoint service ...");
        waypoint_param_client_ =this->create_client<rcl_interfaces::srv::GetParameters>("/waypoint_manager/get_parameters"); //get_parameters is built in, it allows you to get params from any node you want
        while (!waypoint_param_client_->wait_for_service(1s)) RCLCPP_INFO(this->get_logger(), "waypoint_manager/get_parameters service not available, waiting...");


        detections_["person"] = DetectionObject{"person", 0.0, 0.0, 0.0};
        detections_["tent"] = DetectionObject{"tent", 0.0, 0.0, 0.0};

        camera_feed_path_ = resolve_camera_feed_path();

        fetch_mission_indices();
        
    };

private:
    int last_before_rtl_ = 0;
    int next_after_takeoff_ = 0;
    int takeoff_index_ = 0;
    int rtl_index_ = 0;
    int lap_ = 0;
    int waypoint_reached_ = 0;
    int person_wp_ = -1;
    int tent_wp_ = -1;
    int last_nav_before_rtl_ = -1;
    int buffer_wp_ = -1;
    int num_waypoints_ = 0;

    bool wait_to_send_wp_ = true;
    bool waiting_for_processing_ = false;
    bool auto_resumed_ = false;

    set<string> processed_image_names_;
    vector<mavros_msgs::msg::Waypoint> waypoints_;
    map<string, DetectionObject> detections_;

    string camera_feed_path_;

    rclcpp::TimerBase::SharedPtr processing_check_timer_;

    rclcpp::Subscription<interfaces::msg::ImageResult>::SharedPtr image_detection_sub;
    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoint_sub;
    rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr waypoint_reached_sub;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameters_event_sub;
    rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_publisher_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<interfaces::srv::AddWaypoint>::SharedPtr add_wp_client_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr waypoint_param_client_;

    void image_result_cb(const interfaces::msg::ImageResult::SharedPtr msg) {}
    
    void waypoints_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg) {}
    
    void update_waypoint_reached(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
    {
        waypoint_reached_ = msg->wp_seq;

        const int trigger_wp = last_nav_before_rtl_ >= 0 ? last_nav_before_rtl_ : last_before_rtl_;

        if (waypoint_reached_ == trigger_wp && valid_detection("person") && valid_detection("tent") && wait_to_send_wp_);
        {
            double person_lat = detections_["person"].lat;
            double person_lon = detections_["person"].lon;
            double person_alt = ALT;

            double tent_lat = detections_["tent"].lat;
            double tent_lon = detections_["tent"].lon;
            double tent_alt = ALT;
            
            string message = "Detected a person and a tent!";
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
            send_ack(message);

            person_wp_ = last_before_rtl_ + 2;
            tent_wp = last_before_rtl_ + 1;

            vector<WaypointData> waypoint_data ={
                {person_lat, person_lon, person_alt, last_before_rtl_ + 2},
                {tent_lat, tent_lon, tent_alt, last_before_rtl_ + 1}
            };

            send_waypoint_data(waypoint_data);
            wait_to_send_wp = false;
            send_ack("Going to the tent first!");
            RCLCPP_INFO(this->get_logger(), "Waypoint sent. last_before_rtl was: %d", last_before_rtl);

            last_before_rtl_ = -1;
        }
        else if (waypoint_reached_ == trigger_wp && valid_detection("person") || vaid_detection("tent") && wait_to_send_wp_)
        {
            if (valid_detection("tent"))
            {

            double tent_lat = detections_["tent"].lat;
            double tent_lon = detections_["tent"].lon;
            double tent_alt = ALT;
            tent_wp = last_before_rtl_ + 1;

            string message = "Detected a tent!";
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
            send_ack(message);

            vector<WaypointData> waypoint_data ={
                {tent_lat, tent_lon, tent_alt, last_before_rtl_ + 1}
            };

            send_waypoint_data(waypoint_data);
            wait_to_send_wp = false;
            send_ack("Going to the tent only!");
            RCLCPP_INFO(this->get_logger(), "Waypoint sent. last_before_rtl was: %d", last_before_rtl);
            last_before_rtl_ = -1;
            }
            else if (valid_detection("person"))
            {
            double person_let = detections_["tent"].lat;
            double person_lon = detections_["tent"].lon;
            double person_alt = ALT;
            person_wp = last_before_rtl_ + 1;

            string message = "Detected a person!";
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
            send_ack(message);

            vector<WaypointData> waypoint_data ={
                {person_lat, person_lon, person_alt, last_before_rtl_ + 1},
            };

            send_waypoint_data(waypoint_data);
            wait_to_send_wp = false;
            send_ack("Going to the person only!");
            RCLCPP_INFO(this->get_logger(), "Waypoint sent. last_before_rtl was: %d", last_before_rtl);
            last_before_rtl_ = -1;
            }
        }
    }

    bool valid_detection(const string & obj_type) const // first cost means the function will not change the string, second const means we will not change the class object
    {
        auto it = detections_.find(obj_type);
        if (it != detections.end()) // means if the item was found 
        {
            return it->second.confidence > 0.0; // use -> instead of . cus it is an iterator and not an object
        }
        return false;
    }
        
    void fetch_mission_indices() // asks parameters from waypoint.py
    {
        auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();//Creates a service request for ROS 2’s built-in GetParameters service.
        request->names = {
            "num_waypoints",
            "takeoff_index",
            "rtl_index",
            "next_after_takeoff",
            "last_before_rtl"
        };

        auto future = waypoint_param_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get waypoint manager parameters");
            return;
        }

        auto response = future.get();

        if (response->values.size() != request->names.size()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter response size mismatch");
            return;
        }
        // static_cast<int> converts the ROS parameter integer value into a normal C++ int
        num_waypoints_ = static_cast<int>(response->values[0].integer_value);
        takeoff_index_ = static_cast<int>(response->values[1].integer_value);
        rtl_index_ = static_cast<int>(response->values[2].integer_value);
        next_after_takeoff_ = static_cast<int>(response->values[3].integer_value);
        last_before_rtl_ = static_cast<int>(response->values[4].integer_value);
    }

    void parameter_event_cb(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
    {
        if (msg->node == "/waypoint_manager")
        {
            for(const auto & changed_param : msg->changed_parameters) 
            {
                string name = changed_param.name;

                if 
                (
                    name == "num_waypoints" ||
                    name == "takeoff_index" ||
                    name == "rtl_index" ||
                    name == "next_after_takeoff" ||
                    name == "last_before_rtl"
                )
                {
                    this->fetch_mission_indices();
                    break;
                }
            } 
        }
    }

    void send_ack(const string & text)
    {
    }


    void send_waypoint_data(const vector<DetectionWaypoint> & wp_list)
    {
        RCLCPP_INFO(this->get_logger(), "Sending %zu detection waypoint(s)", wp_list.size());
    }

    void change_mode(const string & mode)
    {
        RCLCPP_INFO(this->get_logger(), "Setting mode to %s...", mode.c_str());
    }

    void check_all_images_processed()
    {
    }

    string resolve_camera_feed_path()
    {
        return "";
    }

};


int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainController>());
    rclcpp::shutdown();
    return 0;
}
