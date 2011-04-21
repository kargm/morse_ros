#include <cmath>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <jido_move_head/move_headAction.h>

class move_headAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<jido_move_head::move_headAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  jido_move_head::move_headFeedback feedback_;
  jido_move_head::move_headResult result_;

  // needed for tf and ik calculation
  std::string pan_parent_;
  std::string pan_link_;
  std::string tilt_link_;
  std::string pan_joint_;
  std::string tilt_joint_;
  double success_angle_threshold_;

  tf::TransformListener tf_;
  tf::Stamped<tf::Point> target_in_pan_;
  // Publisher for PTU control messages
  ros::Publisher pub_controller_command_;

public:

  move_headAction(std::string name) :
    as_(nh_, name, boost::bind(&move_headAction::executeCB, this, _1), false),
    action_name_(name)
  {
    // define which frames are used for PTU
    ros::NodeHandle pn("~");
    pn.param("pan_link", pan_link_, std::string("head_pan_link"));
    pn.param("tilt_link", tilt_link_, std::string("head_tilt_link"));
    pn.param("success_angle_threshold", success_angle_threshold_, 0.1);

    // \todo Need to actually look these joints up
    pan_joint_ = "head_pan_joint";
    tilt_joint_ = "head_tilt_joint";

    std::vector<std::string> frames;
    frames.push_back(pan_link_);
    frames.push_back(tilt_link_);

    // In MORSE, PTU is controlled by posting a Vector3
    pub_controller_command_ = nh_.advertise<geometry_msgs::Vector3>("/Jido/Platine", 2);
    as_.start();
  }

  ~move_headAction(void)
  {
  }

  void executeCB(const jido_move_head::move_headGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // Get requested point to look at
    float x = goal->targetPose.point.x;
    float y = goal->targetPose.point.y;
    float z = goal->targetPose.point.z;

    // publish info to the console for the user
    ROS_INFO("Executing, trying to look at point <%f, %f, %f>", x, y, z);

    // gather all the tfs you need

    // Before we do anything, we need to know that name of the pan_link's parent.
    if (pan_parent_.empty())
    {
      for (int i = 0; i < 10; ++i)
          {
            try {
              tf_.getParent(pan_link_, ros::Time(), pan_parent_);
              break;
            }
            catch (const tf::TransformException &ex) {}
            ros::Duration(0.5).sleep();
          }

          if (pan_parent_.empty())
          {
            ROS_ERROR("Could not get parent of %s in the TF tree", pan_link_.c_str());
            //gh.setRejected();
            success = false;
            return;
          }
    }

        std::vector<double> q_goal(2);  // [pan, tilt]
    // Transforms the target point into the pan and tilt links.
    const geometry_msgs::PointStamped &target = goal->targetPose;
    bool ret1 = false, ret2 = false;
    try {
          ros::Time now = ros::Time::now();
          std::string error_msg;
          ret1 = tf_.waitForTransform(pan_parent_, target.header.frame_id, target.header.stamp,
                                     ros::Duration(5.0), ros::Duration(0.01), &error_msg);
          ret2 = tf_.waitForTransform(pan_link_, target.header.frame_id, target.header.stamp,
                                       ros::Duration(5.0), ros::Duration(0.01), &error_msg);

          // Performs IK to determine the desired joint angles

          // Transforms the target into the pan and tilt frames
          tf::Stamped<tf::Point> target_point, target_in_tilt;
          tf::pointStampedMsgToTF(target, target_point);
          tf_.transformPoint(pan_parent_, target_point, target_in_pan_);
          tf_.transformPoint(pan_link_, target_point, target_in_tilt);

          // Computes the desired joint positions.
          q_goal[0] = atan2(target_in_pan_.y(), target_in_pan_.x());
          q_goal[1] = atan2(-target_in_tilt.z(),
                            sqrt(pow(target_in_tilt.x(),2) + pow(target_in_tilt.y(),2)));
    } // end try
    catch(const tf::TransformException &ex)
    {
          ROS_ERROR("Transform failure (%d,%d): %s", ret1, ret2, ex.what());
          //gh.setRejected();
          success = false;
          return;
    } // end catch

    geometry_msgs::Vector3 traj;
    traj.x = 0;
    traj.y = q_goal[1];
    traj.z = q_goal[0];
    pub_controller_command_.publish(traj);

    if (success) {
      result_.resultPanTilt = traj;
      as_.setSucceeded(result_);
    }
    else {
      as_.setPreempted();
    }




    // start executing the action
//    for(int i=1; i<=goal->order; i++)
//    {
//      // check that preempt has not been requested by the client
//      if (as_.isPreemptRequested() || !ros::ok())
//      {
//        ROS_INFO("%s: Preempted", action_name_.c_str());
//        // set the action state to preempted
//        as_.setPreempted();
//        success = false;
//        break;
//      }
//      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
//      // publish the feedback
//      as_.publishFeedback(feedback_);
//      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
//      r.sleep();
//    }
//
//    if(success)
//    {
//      result_.sequence = feedback_.sequence;
//      ROS_INFO("%s: Succeeded", action_name_.c_str());
//      // set the action state to succeeded
//      as_.setSucceeded(result_);
//    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_head");

  move_headAction move(ros::this_node::getName());
  ros::spin();

  return 0;
}
