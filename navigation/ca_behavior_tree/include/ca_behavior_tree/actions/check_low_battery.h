#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>


/*
 * TODO: REPLACE BY THIS APPROACH
 *
 * https://github.com/AndreasZachariae/PeTRA/blob/3aeb742b5b95c441a681997961334f5465494294/petra_central_control/src/conditions/CheckBattery.cpp
 */

int wait_tick = -1;

BT::NodeStatus CheckBattery(BT::TreeNode &self)
{
    static int count = 0;
    if (wait_tick == -1) {
        if (!self.getInput<int>("wait_tick", wait_tick)) {
            wait_tick = 100;
        }
	    count = wait_tick;
        printf("wait_tick: %d\n", wait_tick);
    }
    count--;
    if (count <= 0) {
        printf("Out of battery!!!!!\n");
        return BT::NodeStatus::FAILURE;
    } else {
        printf("Low Power Countdown: %d\n", count);
        return BT::NodeStatus::SUCCESS;
    }
}
