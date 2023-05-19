#include "joy_service.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_node");

  if (ros::console::set_logger_level(
      ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nh;

  ros::Rate rate(1);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  comwise::joy::joy_service joy_srv;
  int step = 0;
  while (ros::ok()) {
    if (step < 1) {
      if(0 == joy_srv.init()) {
        step = 1;
      }
    }

    if (step < 2) {
      if(joy_srv.start()) {
        step = 2;
      }
    }

    rate.sleep();
  }
  ros::waitForShutdown();
  joy_srv.deinit();

  return 0;
}