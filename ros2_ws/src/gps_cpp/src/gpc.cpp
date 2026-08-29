#include <memory>
#include "rclcpp/rclcpp.hpp"

class GPSMavrosServiceNode : public rclcpp : Node
{
    public:
    GPSMavrossServiceNode() : Node (gps_mavros_service_node)
    {
        RCLCPP_INFO(get_logger(), "GPS node started");
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSMavrosServiceNode>());
    rclcpp::shutown();
    return 0;
}