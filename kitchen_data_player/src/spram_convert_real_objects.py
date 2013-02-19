#!/usr/bin/env python
import roslib; roslib.load_manifest('kitchen_data_player')
import rospy
import tf
from std_msgs.msg import String

pub = rospy.Publisher('objects_relative_to_map', String)
tf_listener = tf.TransformListener()
# This helper was developed when I recognized that in thorben real data, object detections
# are in reference to the kinect and not to /map. So the coordinates passed to the update
# of state variable are plain wrong. This helper takes the data relative to Kinect as input, 
# queries TF about the positions relative to /map and outputs a new object detections-string

def convert_coords(data):
    global pub
    global tf_listener
    old_string = data.data
    # split string into objects
    objects = old_string.split(")(")

    new_string = "("
    # now extract object names
    for obj in objects:
        obj_name = obj.replace("(", "").split(" ")[0]
        #print("Querying TF for object: %s"%obj_name)
        try:
            (trans,rot) = tf_listener.lookupTransform('/map', obj_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #print("Found it at: <%s,%s>"%(trans,rot))
        
        new_string += "(%s - %s %s %s %s %s %s %s)"%(obj_name, trans[0], trans[1], trans[2], 
                                                     rot[0], rot[1], rot[2], rot[3])

    new_string += ")"
    print ("\nOLD String was: %s"%old_string)
    print ("NEW String is: %s"%new_string)
    pub.publish(String(new_string))

def listener():
    
    rospy.init_node('objects', anonymous=True)
    rospy.Subscriber("object_detections", String, convert_coords)
    rospy.spin()
    

if __name__ == '__main__':
    listener()
