#include <memory> // this is for smart pointers and make shrea
#include "rclcpp/rclcpp.hpp" // gives access to ros2 features
#include "mavros_msgs/msg/state.hpp" // access to mavros state messages 
#include <functional>

// custom node class
class GPSMavrosServiceNode : public rclcpp::Node
{
    public:
    // constructor, initialize the node
    GPSMavrosServiceNode() : Node("gps_mavros_service_node")
    {
        state_sub_ = create_subscription<mavros_msgs::msg::State>("/mavros/state", 10,
      std::bind(&GPSMavrosServiceNode::state_cb, this, std::placeholders::_1));
        // you can use info, error, debug, etc..
        RCLCPP_INFO(get_logger(), "GPS node started");
    }
    private:
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        connected_ = msg->connected;
    }
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    bool connected_ = false;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSMavrosServiceNode>()); 
    // make_shared<NodeName> creates the node and manages its memory automatically
    // rclcpp::spin keeps the node running and processing the ros2 callbacks
    rclcpp::shutdown();
    return 0;
}