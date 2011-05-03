(in-package mjido-manip-as)

(defvar publisher nil)


(def-exec-callback manip-callback (joints)
  "This function takes in the jointState  message and pursues the action."
  (ros-debug (mjido-manip callback) "entering callback with goal ~a" joints)
        (publish publisher joints)
  (publish-feedback :status 1)
  (succeed-current :success T)
 )


      
(defun start-manip-as ()
  (with-ros-node ("manip-as")
    ;;subscribe the publisher to the topic
    (setf publisher (advertise "Jido/kuka_base" "Sensor_msgs/JointState"))
    (start-action-server 
      ;; action namespace
      "mjido-manip"
      ;; Action Name: packagename/actionfile
      "morse_jido_manipulation_action/arm_manipulation" 
      #'manip-callback)))
