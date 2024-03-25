#pragma once

#include <ixblue_ins_msgs/msg/ins.hpp>

#include <ixblue_stdbin_decoder/data_models/nav_header.h>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

#include <rclcpp/rclcpp.hpp>
//#include <ros/publisher.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include "diagnostics_publisher.h"
#include "tf2/tf2/LinearMath/Matrix3x3.h"
#include <cmath>

class ROSPublisher
{
public:
    ROSPublisher(std::shared_ptr<rclcpp::Node> node);
    void onNewStdBinData(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
                         const ixblue_stdbin_decoder::Data::NavHeader& headerData);

    // Standard ros msgs
    sensor_msgs::msg::Imu::Ptr
    toImuMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
             bool use_compensated_acceleration);
    sensor_msgs::msg::NavSatFix::Ptr
    toNavSatFixMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);
    sensor_msgs::msg::TimeReference::Ptr
    toTimeReference(const ixblue_stdbin_decoder::Data::NavHeader& headerData);

    // iXblue ros msgs
    ixblue_ins_msgs::msg::Ins::Ptr
    toiXInsMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    std::shared_ptr<rclcpp::Node> getNode() const;

protected:
    // Header
    std_msgs::msg::Header getHeader(const ixblue_stdbin_decoder::Data::NavHeader& headerData,
                               const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    // Launch parameters
    std::string frame_id;
    std::string time_source;
    std::string time_origin;
    bool use_compensated_acceleration;

    std::shared_ptr<rclcpp::Node> nh;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr stdImuPublisher;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr stdNavSatFixPublisher;
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr stdTimeReferencePublisher;
    rclcpp::Publisher<ixblue_ins_msgs::msg::Ins>::SharedPtr stdInsPublisher;
    DiagnosticsPublisher diagPub;

    // Utils
    bool useInsAsTimeReference = true;
    bool useUnixAsTimeOrigin = true;

    tf2::Matrix3x3 ixblue2enu_;
    void ixblue2Ros(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
                    geometry_msgs::msg::Quaternion & ros_quat);
};
