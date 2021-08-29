;; Auto-generated. Do not edit!


(when (boundp 'ros_projects::RectangleArea)
  (if (not (find-package "ROS_PROJECTS"))
    (make-package "ROS_PROJECTS"))
  (shadow 'RectangleArea (find-package "ROS_PROJECTS")))
(unless (find-package "ROS_PROJECTS::RECTANGLEAREA")
  (make-package "ROS_PROJECTS::RECTANGLEAREA"))
(unless (find-package "ROS_PROJECTS::RECTANGLEAREAREQUEST")
  (make-package "ROS_PROJECTS::RECTANGLEAREAREQUEST"))
(unless (find-package "ROS_PROJECTS::RECTANGLEAREARESPONSE")
  (make-package "ROS_PROJECTS::RECTANGLEAREARESPONSE"))

(in-package "ROS")





(defclass ros_projects::RectangleAreaRequest
  :super ros::object
  :slots (_width _height ))

(defmethod ros_projects::RectangleAreaRequest
  (:init
   (&key
    ((:width __width) 0.0)
    ((:height __height) 0.0)
    )
   (send-super :init)
   (setq _width (float __width))
   (setq _height (float __height))
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; float32 _width
    4
    ;; float32 _height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ros_projects::RectangleAreaResponse
  :super ros::object
  :slots (_area ))

(defmethod ros_projects::RectangleAreaResponse
  (:init
   (&key
    ((:area __area) 0.0)
    )
   (send-super :init)
   (setq _area (float __area))
   self)
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:serialization-length
   ()
   (+
    ;; float32 _area
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _area
       (sys::poke _area (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _area
     (setq _area (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ros_projects::RectangleArea
  :super ros::object
  :slots ())

(setf (get ros_projects::RectangleArea :md5sum-) "92e9c8f940da77dc3e1bc289f7dd146e")
(setf (get ros_projects::RectangleArea :datatype-) "ros_projects/RectangleArea")
(setf (get ros_projects::RectangleArea :request) ros_projects::RectangleAreaRequest)
(setf (get ros_projects::RectangleArea :response) ros_projects::RectangleAreaResponse)

(defmethod ros_projects::RectangleAreaRequest
  (:response () (instance ros_projects::RectangleAreaResponse :init)))

(setf (get ros_projects::RectangleAreaRequest :md5sum-) "92e9c8f940da77dc3e1bc289f7dd146e")
(setf (get ros_projects::RectangleAreaRequest :datatype-) "ros_projects/RectangleAreaRequest")
(setf (get ros_projects::RectangleAreaRequest :definition-)
      "float32 width
float32 height
---
float32 area

")

(setf (get ros_projects::RectangleAreaResponse :md5sum-) "92e9c8f940da77dc3e1bc289f7dd146e")
(setf (get ros_projects::RectangleAreaResponse :datatype-) "ros_projects/RectangleAreaResponse")
(setf (get ros_projects::RectangleAreaResponse :definition-)
      "float32 width
float32 height
---
float32 area

")



(provide :ros_projects/RectangleArea "92e9c8f940da77dc3e1bc289f7dd146e")


