#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include <memory>
#include <optional>
#include <chrono>

//Messages
#include "mavros_msgs/msg/state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "mavros_msgs/srv/stream_rate.hpp"

using namespace std::chrono_literals;

class GPSNode : public rclcpp::Node
{
public:
    GPSNode() : Node("gps"), connected_(false), latest_gps_(nullptr), latest_yaw_(std::nullopt), latest_connected_(std::nullopt)
    {
        // Subscribers
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>
        ("mavros/state", 10, std::bind(&GPSNode::state_cb, this, std::placeholders::_1));

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>
        ("mavros/global_position/global", rclcpp::SensorDataQoS(), std::bind(&GPSNode::gps_cb, this, std::placeholders::_1));

        yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>
        ("mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(), std::bind(&GPSNode::yaw_cb, this, std::placeholders::_1));

        //Timer
        heartbeat_timer_ = this->create_wall_timer(1s, std::bind(&GPSNode::heartbeat_cb, this));

    } 
private:
    //Callbacks
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg){
        connected_ = msg->connected;
    }

    void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
        latest_gps_ = msg;
        RCLCPP_INFO(this->get_logger(), "GPS: Lat: %.6f, Lon: %.6f, Alt: %.2f", msg->latitude, msg->longitude, msg->altitude);
    }

    void yaw_cb(const std_msgs::msg::Float64::SharedPtr msg){
        latest_yaw_ = msg->data;
    }

    void heartbeat_cb(){
        if(!latest_connected_.has_value() || connected_ != latest_connected_.value()){
            if(connected_){
                RCLCPP_INFO(this->get_logger(), "Heartbeat: Connected to Pixhawk");
            }else{
                RCLCPP_WARN(this->get_logger(), "Heatbeat: Disconnected from Pixhawk");
            }
            latest_connected_ = connected_;
        }
    }

    bool connected_;
    sensor_msgs::msg::NavSatFix::SharedPtr latest_gps_;
    std::optional<double> latest_yaw_;
    std::optional<bool>   latest_connected_;

    // ROS handles
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSNode>());
  rclcpp::shutdown();
  return 0;
}