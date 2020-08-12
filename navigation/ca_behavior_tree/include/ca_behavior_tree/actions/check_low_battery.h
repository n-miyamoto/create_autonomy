#pragma once

#include <memory>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>


class BatteryCharging : public BT::SyncActionNode
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    BatteryCharging(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;
    // BT::NodeStatus onStart() override;
    // BT::NodeStatus onRunning() override;
    // void onHalted() override;

private:
    double battery_percentage_;
    double low_percentage_;

    // The node that will be used for any ROS operations
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber sub_;
};
