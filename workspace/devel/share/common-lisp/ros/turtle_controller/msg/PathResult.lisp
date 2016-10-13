; Auto-generated. Do not edit!


(cl:in-package turtle_controller-msg)


;//! \htmlinclude PathResult.msg.html

(cl:defclass <PathResult> (roslisp-msg-protocol:ros-message)
  ((progress
    :reader progress
    :initarg :progress
    :type cl:float
    :initform 0.0)
   (currentPosition
    :reader currentPosition
    :initarg :currentPosition
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (rightPosition
    :reader rightPosition
    :initarg :rightPosition
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PathResult (<PathResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name turtle_controller-msg:<PathResult> is deprecated: use turtle_controller-msg:PathResult instead.")))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <PathResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtle_controller-msg:progress-val is deprecated.  Use turtle_controller-msg:progress instead.")
  (progress m))

(cl:ensure-generic-function 'currentPosition-val :lambda-list '(m))
(cl:defmethod currentPosition-val ((m <PathResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtle_controller-msg:currentPosition-val is deprecated.  Use turtle_controller-msg:currentPosition instead.")
  (currentPosition m))

(cl:ensure-generic-function 'rightPosition-val :lambda-list '(m))
(cl:defmethod rightPosition-val ((m <PathResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtle_controller-msg:rightPosition-val is deprecated.  Use turtle_controller-msg:rightPosition instead.")
  (rightPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathResult>) ostream)
  "Serializes a message object of type '<PathResult>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rightPosition) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathResult>) istream)
  "Deserializes a message object of type '<PathResult>"
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
    (cl:setf (cl:slot-value msg 'rightPosition) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathResult>)))
  "Returns string type for a message object of type '<PathResult>"
  "turtle_controller/PathResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathResult)))
  "Returns string type for a message object of type 'PathResult"
  "turtle_controller/PathResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathResult>)))
  "Returns md5sum for a message object of type '<PathResult>"
  "29e93df067505daea7e8604a78c2d165")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathResult)))
  "Returns md5sum for a message object of type 'PathResult"
  "29e93df067505daea7e8604a78c2d165")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathResult>)))
  "Returns full string definition for message of type '<PathResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%float32 progress~%float32[] currentPosition~%bool rightPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathResult)))
  "Returns full string definition for message of type 'PathResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%float32 progress~%float32[] currentPosition~%bool rightPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathResult>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'currentPosition) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathResult>))
  "Converts a ROS message object to a list"
  (cl:list 'PathResult
    (cl:cons ':progress (progress msg))
    (cl:cons ':currentPosition (currentPosition msg))
    (cl:cons ':rightPosition (rightPosition msg))
))
