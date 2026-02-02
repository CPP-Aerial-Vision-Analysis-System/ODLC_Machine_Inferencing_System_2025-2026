#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include <memory>
#include <optional>
#include <chrono>
#include <thread>
#include <vector>
#include <string>

// Messages
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/msg/status_text.hpp"

// Services
#include "mavros_msgs/srv/waypoint_pull.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "mavros_msgs/srv/waypoint_clear.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/waypoint_set_current.hpp"

using namespace std::chrono_literals;

class WaypointManager: public rclcpp::Node {
public:
    WaypointManager(): Node("waypoint"), 
    connected_(false), did_initial_pull_(false),
    waypoint_reached_(0), takeoff_index_(-1), rtl_index_(-1), next_after_takeoff_(-1), last_before_rtl_(-1)
    {
        // Subscribers
        state_sub_ = this->create_subscriptiion<mavros_msgs::msg::State>
        ("mavros/state", 10, std::bind(&WaypointManager::state_cb, this, std::placeholders::_1));

        reached_sub_ = this->create_subscription<mavros_msgs::msg::WaypointReached>
        ("mavros/mission/reached", 10, std::bind(&WaypointManager::waypoint_reached_cb, this, std::placeholders::_1));

        list_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>
        ("mavros/mission/waypoints", 10, std::bind(&WaypointManager::waypoint_list_cb, this, std::placeholders::_1));

        // Publishers
        status_pub_ = this->create_publisher<mavros_msgs::msg::StatusText>
        ("mavros/statustext/send", 10);

        // Services
        waypoint_pull_ = this->create_client<mavros_msgs::srv::WaypointPull>
        ("mavros/mission/pull");

        waypoint_push_ = this->create_client<mavros_msgs::srv::WaypointPush>
        ("mavros/mission/push");

        waypoint_clear_ = this->create_client<mavros_msgs::srv::WaypointClear>
        ("mavros/mission/clear");

        set_mode_ = this->create_client<mavros_msgs::srv::SetMode>
        ("mavros/set_mode");

        set_current_ = this->create_client<mavros_msgs::srv::WaypointSetCurrent>
        ("mavros/mission/set_current");

        this->declare_parameter<int>("num_waypoints", 0);
        this->declare_parameter<int>("takeoff_index", -1);
        this->declare_parameter<int>("rtl_index", -1);
        this->declare_parameter<int>("last_before_rtl", -1);
        this->declare_parameter<int>("next_after_takeoff", -1);

        resetIndicies();

        startup_timer_ = this->create_wall_timer(500ms, std::bind(&WaypointManager::startup_cb, this));

    }

    bool isConnected() const { return connected_; }

private:
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg){
        connected_ = msg->connected;
    }

    void waypoint_reached_cb(const mavros_msgs::msg::WaypointReached::SharedPtr msg){
        waypoint_reached_ = msg->wp_seq;
        RCLCPP_INFO(this->get_logger(), "Waypoint Reached: %d", waypoint_reached_);
    }

    void waypoint_list_cb(const mavros_msgs::msg::WaypointList:;SharedPtr msg){
        waypoint_list_ = msg->waypoints;
        RCLCPP_INFO(this->get_logger(), "Received Waypoint List with %zu waypoints", waypoint_list_.size());
    }

    void startup_cb(){
        if(isConnected() && !did_initial_pull_){
            RCLCPP_INFO(this->get_logger(), "Connected to Pixhawk, pulling waypoints...");
            pullWaypoints();
            did_initial_pull_ = true;
        }
    }

    void resetIndicies(){
        takeoff_index_ = -1;
        rtl_index_ = -1;
        next_after_takeoff_ = -1;
        last_before_rtl_ = -1;

        this->set_parameter(rclcpp::Parameter("num_waypoints", 0));
        this->set_parameter(rclcpp::Parameter("takeoff_index", -1));
        this->set_parameter(rclcpp::Parameter("next_after_takeoff", -1));
        this->set_parameter(rclcpp::Parameter("rtl_index", -1));
        this->set_parameter(rclcpp::Parameter("last_before_rtl", -1));
    }

    // Pulling waypoints
    void pull_waypoints(){
        if(!waypoint_pull_->wait_for_service(1s)){
            RCLCPP_INFO(this->get_logger(), "Waiting for waypoint pull request...");
            return
        }

        auto request = std::make_shared<mavros_msgs::srv::WaypointPull::Request>();
        auto cb = [this](rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedFuture future){
            try{
                auto response = future.get();
                if(response->success){
                    RCLCPP_INFO(this->get_logger(), "Successfully pulled %d waypoints", response->wp_received);
                }else{
                    RCLCPP_ERROR(this->get_logger(), "Failed to pull waypoints");
                }
            } catch(const std::exception &e){
                RCLCPP_ERROR(this->get_logger(), "Service called failed: %s", e.what());
            }
        };
        waypoint_pull_client_->async_send_request(request, cb);
        RCLCPP_INFO(this->get_logger(), "Waypoint pull request sent");
    }

    // Pushing waypoints
    void push_waypoints(){
        if(!waypoint_push_->wait_for_service(1s)){
            RCLCPP_INFO(this->get_logger(), "Waiting for waypoint push request...");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
        request->start_index = 0;
        request->waypoints = waypoint_list_.waypoints;

        auto cb = [this](rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture future){
            try{
                auto response = future.get();
                if(response->success){
                    RCLCPP_INFO(this->get_logger(), "Successfully push %d waypoints", response->wp_transferred);
                }else{
                    RCLCPP_ERROR(this->get_logger(), "Failed to pull waypoints");
                }
            } catch(const std::exception &e){
                RCLCPP_ERROR(this->get_logger(), "Service called failed: %s", e.what());
            }
        };
        waypoint_push_->async_send_request(request, cb);
        RCLCPP_INFO(this->get_logger(), "Waypoint push request sent");
    }

    // Clear waypoints
    void clear_waypoints(){
        if(!waypoint_clear_->wait_for_service(1s)){
            RCLCPP_INFO(this->get_logger(), "Waiting for waypoint clear request...");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
        auto cb = [this](rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedFuture future){
            try{
                auto response = future.get();
                if(response->success){
                    RCLCPP_INFO(this->get_logger(), "Successfully cleared waypoints");
                }else{
                    RCLCPP_ERROR(this->get_logger(), "Failed to clear waypoints");
                }
            } catch(const std::exception &e){
                RCLCPP_ERROR(this->get_logger(), "Service called failed: %s", e.what());
            }
        };
        waypoint_clear_->async_send_request(request, cb);
        RCLCPP_INFO(this->get_logger(), "Waypoint clear request sent");
    }

    // Set current waypoint
    void set_current_waypoint(int index){
        if(!set_current_->wait_for_service(1s)){
            RCLCPP_INFO(this->get_logger(), "Waiting for set current waypoint request...");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
        request->wp_seq = index;

        auto cb = [this](rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedFuture future){
            try{
                auto response = future.get();
                if(response->success){
                    RCLCPP_INFO(this->get_logger(), "Successfully set current waypoint to %d", request->wp_seq);
                }else{
                    RCLCPP_ERROR(this->get_logger(), "Failed to set current waypoint");
                }
            } catch(const std::exception &e){
                RCLCPP_ERROR(this->get_logger(), "Service called failed: %s", e.what());
            }
        };

        set_current_->async_send_request(request, cb);
        RCLCPP_INFO(this->get_logger(), "Set current waypoint request sent");
    }

    void insert_new_waypoiints(const std::vector<mavros_msgs::msg::Waypoint> &wp_list){
        try{
            RCLCPP_INFO(this->get_logger(), "Pulling current waypoint list");
            pull_waypoints();

            if (wp_list.empty()){
                RCLCPP_WARN(this->get_logger(), "No waypoints to insert");
                return;
            }
        
            auto original_waypoints = waypoint_list_.waypoints;
            for(const auto &wp: wp_list){
                RCLCPP_INFO(this->get_logger(), "Inserting new waypoint index %d, Lat: %6f, Long: %6f, Alt: %6f", wp.index, wp.x_lat, wp.y_long, wp.z_alt);
                mavros_msgs::msg::Waypoint new_wp;
                new_wp.frame = 3;                 // Global relative altitude
                new_wp.command = 16;                // MAV_CMD_NAV_WAYPOINT
                new_wp.is_current = false;
                new_wp.autocontinue = true;
                new_wp.param1 = 5.0;              // Hold time (seconds)
                new_wp.param2 = 0.0;               // Acceptance radius (m)
                new_wp.param3 = 0.0;               // Pass through
                new_wp.param4 = std::numeric_limits<float>::quiet_NaN();  // Yaw angle
                new_wp.x_lat = wp.lat;
                new_wp.y_long = wp.lon;
                new_wp.z_alt = wp.alt;
                waypoint_list_.waypoints.insert(wp.index, new_wp);
            }
            
            push_waypoints();
            RCLCPP_INFO(this->get_logger(),"Successfully pushed %zu new waypoints", wp_list.size());
            set_current_waypoint(static_cast<uint16_t>(wp_list.front().index));

        } catch (const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Failed to insert new waypoints: %s", e.what());
        }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManager>());
  rclcpp::shutdown();
  return 0;
}