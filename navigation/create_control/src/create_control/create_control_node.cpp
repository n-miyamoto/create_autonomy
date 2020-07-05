#include <chrono>
#include <functional>
#include <ros/callback_queue.h>

#include "create_hw_interface.hpp"

void controlLoop(Create2Interface &hw,
                 controller_manager::ControllerManager &cm,
                 std::chrono::system_clock::time_point &last_time)
{
  std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_time = current_time - last_time;
  ros::Duration elapsed(elapsed_time.count());
  last_time = current_time;

  // hw.read(elapsed);
  // cm.update(ros::Time::now(), elapsed);
  // hw.write();
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "create_hw_interface");
  ros::NodeHandle nh("");

  Create2Interface robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  double rate;
  nh.param<double>("rate", rate, 30.0);

  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner(1, &queue);

  std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
  ros::TimerOptions control_timer(
    ros::Duration(1 / rate),
    std::bind(controlLoop, std::ref(robot), std::ref(cm), std::ref(last_time)),
    &queue);
  // ros::Timer control_loop = nh.createTimer(control_timer);
  spinner.start();

  return 0;
}
