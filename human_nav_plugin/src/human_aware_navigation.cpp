#include <human_aware_navigation/human_aware_navigation.h>

PLUGINLIB_DECLARE_CLASS(human_nav_plugin, HumanAwareNavigation, human_nav_plugin::HumanAwareNavigation, nav_core::BaseGlobalPlanner)

/**
 * A human-aware-navigation-plugin to replace the global planner of nav_core
 */

namespace human_nav_plugin{

  ros::Publisher plan_pub;
  geometry_msgs::PoseStamped humanPose = geometry_msgs::PoseStamped();
  bool pose_init = false;

void humanPoseCallback(const nav_msgs::Odometry& pose)
{
    //ROS_INFO("Found human at: [%g, %g]", pose.pose.pose.position.x, pose.pose.pose.position.y);
    humanPose.pose.position.x = pose.pose.pose.position.x;
    humanPose.pose.position.y = pose.pose.pose.position.y;
    pose_init = true;
}


//Initialise subscriber and spin until first position is received
void initSub(){
  // Subscriber for human pose
  pose_init = false;
  ros::NodeHandle n1;
  ros::Subscriber sub = n1.subscribe("/Human/GPS", 10, humanPoseCallback);
  ros::Rate r(1); // 10 hz
  while (!pose_init) {
    ros::spinOnce();
    ros::Rate r(1); // 10 hz
  }
}

//Initialise publisher for gui_path
void initPub(){
  // Gui_path publisher
  ros::NodeHandle n2;
  plan_pub = n2.advertise<nav_msgs::Path>("global_plan", 1);
}

bool HumanAwareNavigation::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){

  ROS_INFO("[human_navigation] Getting start point (%g,%g) and goal point (%g,%g)",
               start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  plan.clear();

  // Initialise Subscriber to get human pose
  initSub();
  // Initialise Publisher for gui_plan
  initPub();
  ROS_INFO("[human_navigation] Found human at: [%g, %g]", humanPose.pose.position.x, humanPose.pose.position.y);
  ROS_INFO("[human_navigation] Starting human-friendly planner...");

  // XMLRPC settings
  int port = 7011; // 7011 standard port
  const char* host = "lapbeetz6";
  Moved3dXmlRpcClient* c =  new Moved3dXmlRpcClient(host, port);

  AbsoluteTrajectory waypoints; // result
  Human humans[MHPD_MAX_HUMANS];
  Human h1;
  h1.x = humanPose.pose.position.x;
  h1.y = humanPose.pose.position.y;
  h1.az = 0.0;
  h1.pose = STANDING_TRANSPARENT;
  h1.locked = 0;
  h1.exists = 1;
  h1.lastMoveTimeStampSecs = 0;
  humans[0] = h1;

  int human_no = 1;
  // Query tp create plan
  c->planPath(start.pose.position.x, start.pose.position.y, 0.0, // pos
                      goal.pose.position.x, goal.pose.position.y, 0.0, // goal
                      humans, human_no, // humans
                      &waypoints,
                      0);

  std::cout << "[human_navigation:move3d] Got path with costs " << waypoints.costs;

  // init gui path
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = "map";
  gui_path.header.stamp  = ros::Time::now();

  // fill plan with waypoints
  for(int i=0; i<waypoints.numberOfSegments; i++) {
    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.header.stamp = ros::Time::now();
    pose_tmp.header.frame_id = "map";
    pose_tmp.pose.position.x = waypoints.seg[i].x;
    pose_tmp.pose.position.y = waypoints.seg[i].y;
    ROS_INFO("[human_navigation] Added waypoint: (%g,%g)",
                   pose_tmp.pose.position.x, pose_tmp.pose.position.y);
    plan.push_back(pose_tmp);
    // fill graph for visualization
    gui_path.poses.push_back(pose_tmp);
  }
  // post path message for visualization
  plan_pub.publish(gui_path);
  return true;
}
};
