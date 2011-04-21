#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <jido_move_head/move_headAction.h>
#include <geometry_msgs/PointStamped.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_move_head");

  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<jido_move_head::move_headAction> ac("move_head", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  jido_move_head::move_headGoal goal;
  goal.targetPose.header.frame_id = "map";
  goal.targetPose.point.x = 8;
  goal.targetPose.point.y = 2;
  goal.targetPose.point.z = 1;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
