#pragma once

#include <ixblue_ins_msgs/msg/ins.hpp>

#include <ixblue_stdbin_decoder/data_models/nav_header.h>
#include <ixblue_stdbin_decoder/data_models/stdbin.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include "diagnostics_publisher.h"
#include "tf2/tf2/LinearMath/Matrix3x3.h"
#include <sensor_msgs/msg/range.hpp>
#include <cmath>

class ROSPublisher
{
public:
    ROSPublisher(std::shared_ptr<rclcpp::Node> node);
    void onNewStdBinData(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
                         const ixblue_stdbin_decoder::Data::NavHeader& headerData);

    // Standard ros msgs
    sensor_msgs::msg::Imu::SharedPtr
    toImuMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData,
             bool use_compensated_acceleration);

    sensor_msgs::msg::NavSatFix::SharedPtr
    toNavSatFixMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    sensor_msgs::msg::TimeReference::SharedPtr
    toTimeReference(const ixblue_stdbin_decoder::Data::NavHeader& headerData);

    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr
    toTwistMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    sensor_msgs::msg::Range::SharedPtr
    dvlToRangeMsg(const ixblue_stdbin_decoder::Data::BinaryNav& navData);

    // iXblue ros msgs
    ixblue_ins_msgs::msg::Ins::SharedPtr
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
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr stdTwistPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr rangePublisher;
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
