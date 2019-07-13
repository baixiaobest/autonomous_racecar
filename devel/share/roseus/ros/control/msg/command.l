;; Auto-generated. Do not edit!


(when (boundp 'control::command)
  (if (not (find-package "CONTROL"))
    (make-package "CONTROL"))
  (shadow 'command (find-package "CONTROL")))
(unless (find-package "CONTROL::COMMAND")
  (make-package "CONTROL::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude command.msg.html


(defclass control::command
  :super ros::object
  :slots (_throttle _steering ))

(defmethod control::command
  (:init
   (&key
    ((:throttle __throttle) 0.0)
    ((:steering __steering) 0.0)
    )
   (send-super :init)
   (setq _throttle (float __throttle))
   (setq _steering (float __steering))
   self)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:serialization-length
   ()
   (+
    ;; float64 _throttle
    8
    ;; float64 _steering
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steering
       (sys::poke _steering (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steering
     (setq _steering (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get control::command :md5sum-) "39f463d271c2ca10c14182802c72c029")
(setf (get control::command :datatype-) "control/command")
(setf (get control::command :definition-)
      "# Throttle in percentage. Range of -100 to 100.
# Negative number means braking.
float64 throttle
# Steering angle of the vehicle. 
# Range of -100 to 100.
# Postive means left turn, negative means right turn.
float64 steering

")



(provide :control/command "39f463d271c2ca10c14182802c72c029")


