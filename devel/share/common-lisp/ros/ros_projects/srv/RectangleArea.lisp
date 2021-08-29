; Auto-generated. Do not edit!


(cl:in-package ros_projects-srv)


;//! \htmlinclude RectangleArea-request.msg.html

(cl:defclass <RectangleArea-request> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0))
)

(cl:defclass RectangleArea-request (<RectangleArea-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RectangleArea-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RectangleArea-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_projects-srv:<RectangleArea-request> is deprecated: use ros_projects-srv:RectangleArea-request instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <RectangleArea-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_projects-srv:width-val is deprecated.  Use ros_projects-srv:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <RectangleArea-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_projects-srv:height-val is deprecated.  Use ros_projects-srv:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RectangleArea-request>) ostream)
  "Serializes a message object of type '<RectangleArea-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RectangleArea-request>) istream)
  "Deserializes a message object of type '<RectangleArea-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RectangleArea-request>)))
  "Returns string type for a service object of type '<RectangleArea-request>"
  "ros_projects/RectangleAreaRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RectangleArea-request)))
  "Returns string type for a service object of type 'RectangleArea-request"
  "ros_projects/RectangleAreaRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RectangleArea-request>)))
  "Returns md5sum for a message object of type '<RectangleArea-request>"
  "92e9c8f940da77dc3e1bc289f7dd146e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RectangleArea-request)))
  "Returns md5sum for a message object of type 'RectangleArea-request"
  "92e9c8f940da77dc3e1bc289f7dd146e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RectangleArea-request>)))
  "Returns full string definition for message of type '<RectangleArea-request>"
  (cl:format cl:nil "float32 width~%float32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RectangleArea-request)))
  "Returns full string definition for message of type 'RectangleArea-request"
  (cl:format cl:nil "float32 width~%float32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RectangleArea-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RectangleArea-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RectangleArea-request
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
;//! \htmlinclude RectangleArea-response.msg.html

(cl:defclass <RectangleArea-response> (roslisp-msg-protocol:ros-message)
  ((area
    :reader area
    :initarg :area
    :type cl:float
    :initform 0.0))
)

(cl:defclass RectangleArea-response (<RectangleArea-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RectangleArea-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RectangleArea-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_projects-srv:<RectangleArea-response> is deprecated: use ros_projects-srv:RectangleArea-response instead.")))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <RectangleArea-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_projects-srv:area-val is deprecated.  Use ros_projects-srv:area instead.")
  (area m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RectangleArea-response>) ostream)
  "Serializes a message object of type '<RectangleArea-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RectangleArea-response>) istream)
  "Deserializes a message object of type '<RectangleArea-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RectangleArea-response>)))
  "Returns string type for a service object of type '<RectangleArea-response>"
  "ros_projects/RectangleAreaResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RectangleArea-response)))
  "Returns string type for a service object of type 'RectangleArea-response"
  "ros_projects/RectangleAreaResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RectangleArea-response>)))
  "Returns md5sum for a message object of type '<RectangleArea-response>"
  "92e9c8f940da77dc3e1bc289f7dd146e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RectangleArea-response)))
  "Returns md5sum for a message object of type 'RectangleArea-response"
  "92e9c8f940da77dc3e1bc289f7dd146e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RectangleArea-response>)))
  "Returns full string definition for message of type '<RectangleArea-response>"
  (cl:format cl:nil "float32 area~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RectangleArea-response)))
  "Returns full string definition for message of type 'RectangleArea-response"
  (cl:format cl:nil "float32 area~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RectangleArea-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RectangleArea-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RectangleArea-response
    (cl:cons ':area (area msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RectangleArea)))
  'RectangleArea-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RectangleArea)))
  'RectangleArea-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RectangleArea)))
  "Returns string type for a service object of type '<RectangleArea>"
  "ros_projects/RectangleArea")