#!/usr/bin/env python  
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('tf')
roslib.load_manifest('roscpp')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('rosgraph_msgs')
roslib.load_manifest('geometry_msgs')
import rospy
import time
import math
import tf
import geometry_msgs
import std_msgs
import nav_msgs
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3Stamped

# Global variables to store the robot starting position in the /map frame for building the /odom frame
odom_init = 0
odom_pose_x = 0
odom_pose_y = 0
odom_pose_z = 0
odom_quat_x = 0
odom_quat_y = 0
odom_quat_z = 0
odom_quat_w = 0

# Global variables for odometry_msg
odom_msg_x = 0
odom_msg_y = 0
odom_msg_yaw = 0
last_time = 0
pub = rospy.Publisher('/odom', Odometry)
odom_msg_init = 0

def handle_human_pose(msg, robotname):
    now = rospy.Time.now()
    br2 = tf.TransformBroadcaster()
    br2.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     robotname,
                     "/map")
                     
def handle_odom_tf(msg, robotname):
    br3 = tf.TransformBroadcaster()
  
    global odom_init
    global odom_pose_x
    global odom_pose_y
    global odom_pose_z
    global odom_quat_x
    global odom_quat_y
    global odom_quat_z
    global odom_quat_w

    # To obtain the initial point of the odom-frame, the global coordinates are set with the first message
    if odom_init == 0:
        odom_pose_x = msg.pose.pose.position.x
        odom_pose_y = msg.pose.pose.position.y
        odom_pose_z = msg.pose.pose.position.z
        odom_quat_x = msg.pose.pose.orientation.x
        odom_quat_y = msg.pose.pose.orientation.y
        odom_quat_z = msg.pose.pose.orientation.z
        odom_quat_w = msg.pose.pose.orientation.w
        odom_init = 1

    # publish the odom init
    br3.sendTransform((odom_pose_x, odom_pose_y, odom_pose_z),
                    (odom_quat_x, odom_quat_y, odom_quat_z, odom_quat_w),
                    rospy.Time.now(),
                    "/odom",
                    "/map")
    # transformation bewteen map and robot frame
    br3.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "/base_footprint",
                     "/map")

def handle_odom_msg(msg, robotname):

    global pub
    global last_time
    global odom_msg_init
    global odom_msg_x
    global odom_msg_y
    global odom_msg_yaw

    # If not yet initialized, set last_time to timestamp of current message
    if odom_msg_init == 0:
        last_time = msg.header.stamp.to_sec()
        odom_msg_init = 1
        
    odometry = Odometry()

    # Odometry positions are calculated by adding the deltas to the last pose
    odom_msg_x = odom_msg_x + msg.pose.position.x
    odom_msg_y = odom_msg_y + msg.pose.position.y

    # Posestamped gets orientation as Quaternion, so transformation is necessary
    odom_msg_orientation_euler = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    odom_msg_yaw = odom_msg_yaw + odom_msg_orientation_euler[2]
    odom_quat =  tf.transformations.quaternion_from_euler(0, 0, odom_msg_yaw)

    # Calculate velocities
    current_time = msg.header.stamp.to_sec()
    dt = current_time - last_time
    last_time = current_time
    
    
    dyaw = odom_msg_orientation_euler[2]
    
    dx = msg.pose.position.x
    dy = msg.pose.position.y
    distance = math.sqrt(dx**2 + dy**2)
    
    # ROS wants angle velocities in rad/s, so convertion necessary
    dyaw = dyaw * (math.pi / 180)
    
    # in first iteration division by 0 occurs
    if dt != 0:
        # calculate velocities
        vx = distance / dt
        vyaw = dyaw / dt
    else:
        vx = 0
        
        vyaw = 0

    # Build odometry message
    odometry.pose.pose.position.x = odom_msg_x
    odometry.pose.pose.position.y = odom_msg_y
    odometry.pose.pose.position.z = 0 # only 2D navigation here
    odometry.pose.pose.orientation.x = odom_quat[0]
    odometry.pose.pose.orientation.y = odom_quat[1]
    odometry.pose.pose.orientation.z = odom_quat[2]
    odometry.pose.pose.orientation.w = odom_quat[3]

    odometry.child_frame_id = "base_link"
    odometry.twist.twist.linear.x = vx
    odometry.twist.twist.linear.y = 0
    odometry.twist.twist.linear.z = 0
    odometry.twist.twist.angular.x = 0
    odometry.twist.twist.angular.y = 0
    odometry.twist.twist.angular.z = vyaw

    odometry.header.stamp = msg.header.stamp
    odometry.header.frame_id = "odom"

    pub.publish(odometry) 

if __name__ == '__main__':
    rospy.init_node('morse_tf_broadcaster')
    
    # Initialize odom-frame with robot starting-position
    rospy.Subscriber('/Jido/Pose_sensor',
                     nav_msgs.msg.Odometry,
                     handle_odom_tf,
                     '/Jido/Pose_sensor')

    # Handle odometry information using Odometry sensor of Jido
    rospy.Subscriber('/Jido/Odometry',
                     geometry_msgs.msg.PoseStamped,
                     handle_odom_msg,
                     '/Jido/Pose_sensor')

    # Ground-truth frame for Human Position
    rospy.Subscriber('/Human/GPS',
                     nav_msgs.msg.Odometry,
                     handle_human_pose, 
                     '/human_truepose')

    rospy.spin()
