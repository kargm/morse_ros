(asdf:defsystem morse-jido-manip-as
   :depends-on (actionlib
               sensor_msgs-msg)
  :components
  ((:module "mjido-manip-as"
            :components
            ((:file "package")
             (:file "morse-jido-manip-action-server" :depends-on ("package"))))))