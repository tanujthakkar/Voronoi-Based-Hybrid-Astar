;; Auto-generated. Do not edit!


(when (boundp 'ridgeback_msgs::Fans)
  (if (not (find-package "RIDGEBACK_MSGS"))
    (make-package "RIDGEBACK_MSGS"))
  (shadow 'Fans (find-package "RIDGEBACK_MSGS")))
(unless (find-package "RIDGEBACK_MSGS::FANS")
  (make-package "RIDGEBACK_MSGS::FANS"))

(in-package "ROS")
;;//! \htmlinclude Fans.msg.html


(intern "*EQUIPMENT_BAY_INTAKE*" (find-package "RIDGEBACK_MSGS::FANS"))
(shadow '*EQUIPMENT_BAY_INTAKE* (find-package "RIDGEBACK_MSGS::FANS"))
(defconstant ridgeback_msgs::Fans::*EQUIPMENT_BAY_INTAKE* 0)
(intern "*EQUIPMENT_BAY_EXHAUST*" (find-package "RIDGEBACK_MSGS::FANS"))
(shadow '*EQUIPMENT_BAY_EXHAUST* (find-package "RIDGEBACK_MSGS::FANS"))
(defconstant ridgeback_msgs::Fans::*EQUIPMENT_BAY_EXHAUST* 1)
(intern "*CHARGER_BAY_INTAKE*" (find-package "RIDGEBACK_MSGS::FANS"))
(shadow '*CHARGER_BAY_INTAKE* (find-package "RIDGEBACK_MSGS::FANS"))
(defconstant ridgeback_msgs::Fans::*CHARGER_BAY_INTAKE* 2)
(intern "*CHARGER_BAY_EXHAUST*" (find-package "RIDGEBACK_MSGS::FANS"))
(shadow '*CHARGER_BAY_EXHAUST* (find-package "RIDGEBACK_MSGS::FANS"))
(defconstant ridgeback_msgs::Fans::*CHARGER_BAY_EXHAUST* 3)
(intern "*USER_BAY_INTAKE*" (find-package "RIDGEBACK_MSGS::FANS"))
(shadow '*USER_BAY_INTAKE* (find-package "RIDGEBACK_MSGS::FANS"))
(defconstant ridgeback_msgs::Fans::*USER_BAY_INTAKE* 4)
(intern "*USER_BAY_EXHAUST*" (find-package "RIDGEBACK_MSGS::FANS"))
(shadow '*USER_BAY_EXHAUST* (find-package "RIDGEBACK_MSGS::FANS"))
(defconstant ridgeback_msgs::Fans::*USER_BAY_EXHAUST* 5)
(intern "*FAN_OFF*" (find-package "RIDGEBACK_MSGS::FANS"))
(shadow '*FAN_OFF* (find-package "RIDGEBACK_MSGS::FANS"))
(defconstant ridgeback_msgs::Fans::*FAN_OFF* 0)
(intern "*FAN_ON_HIGH*" (find-package "RIDGEBACK_MSGS::FANS"))
(shadow '*FAN_ON_HIGH* (find-package "RIDGEBACK_MSGS::FANS"))
(defconstant ridgeback_msgs::Fans::*FAN_ON_HIGH* 1)
(intern "*FAN_ON_LOW*" (find-package "RIDGEBACK_MSGS::FANS"))
(shadow '*FAN_ON_LOW* (find-package "RIDGEBACK_MSGS::FANS"))
(defconstant ridgeback_msgs::Fans::*FAN_ON_LOW* 2)
(defclass ridgeback_msgs::Fans
  :super ros::object
  :slots (_fans ))

(defmethod ridgeback_msgs::Fans
  (:init
   (&key
    ((:fans __fans) (make-array 6 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _fans __fans)
   self)
  (:fans
   (&optional __fans)
   (if __fans (setq _fans __fans)) _fans)
  (:serialization-length
   ()
   (+
    ;; uint8[6] _fans
    (* 1    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[6] _fans
     (princ _fans s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[6] _fans
   (setq _fans (make-array 6 :element-type :char))
   (replace _fans buf :start2 ptr-) (incf ptr- 6)
   ;;
   self)
  )

(setf (get ridgeback_msgs::Fans :md5sum-) "d529aec610975f8df12d912730064bbf")
(setf (get ridgeback_msgs::Fans :datatype-) "ridgeback_msgs/Fans")
(setf (get ridgeback_msgs::Fans :definition-)
      "# Location of fans.
uint8 EQUIPMENT_BAY_INTAKE=0
uint8 EQUIPMENT_BAY_EXHAUST=1
uint8 CHARGER_BAY_INTAKE=2
uint8 CHARGER_BAY_EXHAUST=3
uint8 USER_BAY_INTAKE=4
uint8 USER_BAY_EXHAUST=5

uint8 FAN_OFF=0
uint8 FAN_ON_HIGH=1
uint8 FAN_ON_LOW=2
uint8[6] fans

")



(provide :ridgeback_msgs/Fans "d529aec610975f8df12d912730064bbf")


