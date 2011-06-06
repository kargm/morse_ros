(in-package mjido-manip-as)

(defvar publisher nil)


(def-exec-callback manip-callback (joints)
  "This function takes in the jointState  message and pursues the action."
  (ros-debug (mjido-manip-as callback) "entering callback with goal ~a" joints)
  (publish publisher joints)
  (roslisp:ros-info mjido-manip-as "Get Action Client.")
  ;; TODO read current pose, give feedback in loop
  ;; (publish-feedback :status 1)
  (succeed-current :success T)
 )


      
(defun start-manip-as ()
  (with-ros-node ("manip_as")
    ;;subscribe the publisher to the topic
    (setf publisher (advertise "Jido/kuka_base" "sensor_msgs/JointState"))
    (start-action-server 
      ;; action namespace
      "mjido_manip"
      ;; Action Name: packagename/actionfile
      "morse_jido_manipulation_action/ReachObjectAction" 
      #'manip-callback)))
