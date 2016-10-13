; Auto-generated. Do not edit!


(cl:in-package turtle_controller-msg)


;//! \htmlinclude PathFeedback.msg.html

(cl:defclass <PathFeedback> (roslisp-msg-protocol:ros-message)
  ((progress
    :reader progress
    :initarg :progress
    :type cl:float
    :initform 0.0)
   (currentPosition
    :reader currentPosition
    :initarg :currentPosition
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass PathFeedback (<PathFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name turtle_controller-msg:<PathFeedback> is deprecated: use turtle_controller-msg:PathFeedback instead.")))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <PathFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtle_controller-msg:progress-val is deprecated.  Use turtle_controller-msg:progress instead.")
  (progress m))

(cl:ensure-generic-function 'currentPosition-val :lambda-list '(m))
(cl:defmethod currentPosition-val ((m <PathFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtle_controller-msg:currentPosition-val is deprecated.  Use turtle_controller-msg:currentPosition instead.")
  (currentPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathFeedback>) ostream)
  "Serializes a message object of type '<PathFeedback>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'progress))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'currentPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'currentPosition))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathFeedback>) istream)
  "Deserializes a message object of type '<PathFeedback>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'progress) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'currentPosition) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'currentPosition)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathFeedback>)))
  "Returns string type for a message object of type '<PathFeedback>"
  "turtle_controller/PathFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathFeedback)))
  "Returns string type for a message object of type 'PathFeedback"
  "turtle_controller/PathFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathFeedback>)))
  "Returns md5sum for a message object of type '<PathFeedback>"
  "dc3da8d59bf639e439e08c2db57db40f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathFeedback)))
  "Returns md5sum for a message object of type 'PathFeedback"
  "dc3da8d59bf639e439e08c2db57db40f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathFeedback>)))
  "Returns full string definition for message of type '<PathFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float32 progress~%float32[] currentPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathFeedback)))
  "Returns full string definition for message of type 'PathFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float32 progress~%float32[] currentPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathFeedback>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'currentPosition) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'PathFeedback
    (cl:cons ':progress (progress msg))
    (cl:cons ':currentPosition (currentPosition msg))
))