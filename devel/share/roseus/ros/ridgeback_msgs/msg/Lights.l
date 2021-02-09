;; Auto-generated. Do not edit!


(when (boundp 'ridgeback_msgs::Lights)
  (if (not (find-package "RIDGEBACK_MSGS"))
    (make-package "RIDGEBACK_MSGS"))
  (shadow 'Lights (find-package "RIDGEBACK_MSGS")))
(unless (find-package "RIDGEBACK_MSGS::LIGHTS")
  (make-package "RIDGEBACK_MSGS::LIGHTS"))

(in-package "ROS")
;;//! \htmlinclude Lights.msg.html


(intern "*LIGHTS_FRONT_LEFT_UPPER*" (find-package "RIDGEBACK_MSGS::LIGHTS"))
(shadow '*LIGHTS_FRONT_LEFT_UPPER* (find-package "RIDGEBACK_MSGS::LIGHTS"))
(defconstant ridgeback_msgs::Lights::*LIGHTS_FRONT_LEFT_UPPER* 0)
(intern "*LIGHTS_FRONT_LEFT_LOWER*" (find-package "RIDGEBACK_MSGS::LIGHTS"))
(shadow '*LIGHTS_FRONT_LEFT_LOWER* (find-package "RIDGEBACK_MSGS::LIGHTS"))
(defconstant ridgeback_msgs::Lights::*LIGHTS_FRONT_LEFT_LOWER* 1)
(intern "*LIGHTS_FRONT_RIGHT_UPPER*" (find-package "RIDGEBACK_MSGS::LIGHTS"))
(shadow '*LIGHTS_FRONT_RIGHT_UPPER* (find-package "RIDGEBACK_MSGS::LIGHTS"))
(defconstant ridgeback_msgs::Lights::*LIGHTS_FRONT_RIGHT_UPPER* 2)
(intern "*LIGHTS_FRONT_RIGHT_LOWER*" (find-package "RIDGEBACK_MSGS::LIGHTS"))
(shadow '*LIGHTS_FRONT_RIGHT_LOWER* (find-package "RIDGEBACK_MSGS::LIGHTS"))
(defconstant ridgeback_msgs::Lights::*LIGHTS_FRONT_RIGHT_LOWER* 3)
(intern "*LIGHTS_REAR_LEFT_UPPER*" (find-package "RIDGEBACK_MSGS::LIGHTS"))
(shadow '*LIGHTS_REAR_LEFT_UPPER* (find-package "RIDGEBACK_MSGS::LIGHTS"))
(defconstant ridgeback_msgs::Lights::*LIGHTS_REAR_LEFT_UPPER* 4)
(intern "*LIGHTS_REAR_LEFT_LOWER*" (find-package "RIDGEBACK_MSGS::LIGHTS"))
(shadow '*LIGHTS_REAR_LEFT_LOWER* (find-package "RIDGEBACK_MSGS::LIGHTS"))
(defconstant ridgeback_msgs::Lights::*LIGHTS_REAR_LEFT_LOWER* 5)
(intern "*LIGHTS_REAR_RIGHT_UPPER*" (find-package "RIDGEBACK_MSGS::LIGHTS"))
(shadow '*LIGHTS_REAR_RIGHT_UPPER* (find-package "RIDGEBACK_MSGS::LIGHTS"))
(defconstant ridgeback_msgs::Lights::*LIGHTS_REAR_RIGHT_UPPER* 6)
(intern "*LIGHTS_REAR_RIGHT_LOWER*" (find-package "RIDGEBACK_MSGS::LIGHTS"))
(shadow '*LIGHTS_REAR_RIGHT_LOWER* (find-package "RIDGEBACK_MSGS::LIGHTS"))
(defconstant ridgeback_msgs::Lights::*LIGHTS_REAR_RIGHT_LOWER* 7)
(defclass ridgeback_msgs::Lights
  :super ros::object
  :slots (_lights ))

(defmethod ridgeback_msgs::Lights
  (:init
   (&key
    ((:lights __lights) (let (r) (dotimes (i 8) (push (instance ridgeback_msgs::RGB :init) r)) r))
    )
   (send-super :init)
   (setq _lights __lights)
   self)
  (:lights
   (&rest __lights)
   (if (keywordp (car __lights))
       (send* _lights __lights)
     (progn
       (if __lights (setq _lights (car __lights)))
       _lights)))
  (:serialization-length
   ()
   (+
    ;; ridgeback_msgs/RGB[8] _lights
    (apply #'+ (send-all _lights :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ridgeback_msgs/RGB[8] _lights
     (dolist (elem _lights)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ridgeback_msgs/RGB[8] _lights
   (dotimes (i 8)
     (send (elt _lights i) :deserialize buf ptr-) (incf ptr- (send (elt _lights i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get ridgeback_msgs::Lights :md5sum-) "2c68505ba4cf8e160d2760ed01777bc7")
(setf (get ridgeback_msgs::Lights :datatype-) "ridgeback_msgs/Lights")
(setf (get ridgeback_msgs::Lights :definition-)
      "# Represents a command for the 8 RGB body lights on Ridgeback.

uint8 LIGHTS_FRONT_LEFT_UPPER=0
uint8 LIGHTS_FRONT_LEFT_LOWER=1
uint8 LIGHTS_FRONT_RIGHT_UPPER=2
uint8 LIGHTS_FRONT_RIGHT_LOWER=3
uint8 LIGHTS_REAR_LEFT_UPPER=4
uint8 LIGHTS_REAR_LEFT_LOWER=5
uint8 LIGHTS_REAR_RIGHT_UPPER=6
uint8 LIGHTS_REAR_RIGHT_LOWER=7

ridgeback_msgs/RGB[8] lights

================================================================================
MSG: ridgeback_msgs/RGB
# Represents the intensity of a single RGB LED, either reported or commanded.

float32 red
float32 green
float32 blue

")



(provide :ridgeback_msgs/Lights "2c68505ba4cf8e160d2760ed01777bc7")


