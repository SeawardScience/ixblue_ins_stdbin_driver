#include "udp_listener.h"
#include "ros_publisher.h"
//#include <ros/ros.h>

int main(int argc, char* argv[])
{
    //ros::init(argc, argv, "ixblue_ins_driver");
    //ros::NodeHandle nh("~");

    auto node =  std::make_shared<ROSPublisher>();

    std::string ip;
    int udp_port;

    node->declare_parameter("udp_port", 8200);
    node->get_parameter("udp_port", udp_port);

    node->declare_parameter("ip", "0.0.0.0");
    node->get_parameter("ip", ip);


    RCLCPP_INFO(node->get_logger(),"UDP port : %d", udp_port);
    RCLCPP_INFO(node->get_logger(),"IP adress : %s", ip.c_str());

    if(udp_port > std::numeric_limits<uint16_t>::max())
    {
        RCLCPP_ERROR_STREAM(node->get_logger(),
            "UDP Port can't be greater than : " << std::numeric_limits<uint16_t>::max());
        return -1;
    }

    UDPListener udpListener(ip, static_cast<uint16_t>(udp_port));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
