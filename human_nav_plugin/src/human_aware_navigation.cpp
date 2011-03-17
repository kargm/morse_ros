#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include <sstream>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_REGISTER_CLASS(HumanAwareNavigation, human_aware_navigation::HumanAwareNavigation, nav_core::BaseGlobalPlanner);
/**
 * A human-aware-navigation-plugin to replace the global planner of nav_core
 */

namespace human_aware_navigation{

int HumanAwareNavigation()
{

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    // Create dummy messages
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
};
