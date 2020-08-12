#include <ca_behavior_tree/actions/check_low_battery.h>


BatteryCharging::BatteryCharging(const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
, battery_percentage_(100.0)
, low_percentage_(0.0)
{
    nh_ = std::make_shared<ros::NodeHandle>("~");
    sub_ = nh_->subscribe<sensor_msgs::BatteryState>(
        "BatteryState", 10,
        [&](sensor_msgs::BatteryState::ConstPtr msg) {
        battery_percentage_ = msg->percentage;
    });
}

BT::NodeStatus BatteryCharging::tick()
{
    if (battery_percentage_ <= low_percentage_)
    {
        ROS_WARN_STREAM("[BatteryState] Battery less than " << low_percentage_ << "%");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}
