;; Auto-generated. Do not edit!


(when (boundp 'ridgeback_msgs::RGB)
  (if (not (find-package "RIDGEBACK_MSGS"))
    (make-package "RIDGEBACK_MSGS"))
  (shadow 'RGB (find-package "RIDGEBACK_MSGS")))
(unless (find-package "RIDGEBACK_MSGS::RGB")
  (make-package "RIDGEBACK_MSGS::RGB"))

(in-package "ROS")
;;//! \htmlinclude RGB.msg.html


(defclass ridgeback_msgs::RGB
  :super ros::object
  :slots (_red _green _blue ))

(defmethod ridgeback_msgs::RGB
  (:init
   (&key
    ((:red __red) 0.0)
    ((:green __green) 0.0)
    ((:blue __blue) 0.0)
    )
   (send-super :init)
   (setq _red (float __red))
   (setq _green (float __green))
   (setq _blue (float __blue))
   self)
  (:red
   (&optional __red)
   (if __red (setq _red __red)) _red)
  (:green
   (&optional __green)
   (if __green (setq _green __green)) _green)
  (:blue
   (&optional __blue)
   (if __blue (setq _blue __blue)) _blue)
  (:serialization-length
   ()
   (+
    ;; float32 _red
    4
    ;; float32 _green
    4
    ;; float32 _blue
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _red
       (sys::poke _red (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _green
       (sys::poke _green (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _blue
       (sys::poke _blue (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _red
     (setq _red (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _green
     (setq _green (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _blue
     (setq _blue (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ridgeback_msgs::RGB :md5sum-) "fc84fca2ee69069d6d5c4147f9b2e33a")
(setf (get ridgeback_msgs::RGB :datatype-) "ridgeback_msgs/RGB")
(setf (get ridgeback_msgs::RGB :definition-)
      "# Represents the intensity of a single RGB LED, either reported or commanded.

float32 red
float32 green
float32 blue

")



(provide :ridgeback_msgs/RGB "fc84fca2ee69069d6d5c4147f9b2e33a")


