; Auto-generated. Do not edit!


(cl:in-package compression-msg)


;//! \htmlinclude msg_features.msg.html

(cl:defclass <msg_features> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (tframe
    :reader tframe
    :initarg :tframe
    :type cl:float
    :initform 0.0)
   (nrobotid
    :reader nrobotid
    :initarg :nrobotid
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass msg_features (<msg_features>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msg_features>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msg_features)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compression-msg:<msg_features> is deprecated: use compression-msg:msg_features instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <msg_features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compression-msg:header-val is deprecated.  Use compression-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'tframe-val :lambda-list '(m))
(cl:defmethod tframe-val ((m <msg_features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compression-msg:tframe-val is deprecated.  Use compression-msg:tframe instead.")
  (tframe m))

(cl:ensure-generic-function 'nrobotid-val :lambda-list '(m))
(cl:defmethod nrobotid-val ((m <msg_features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compression-msg:nrobotid-val is deprecated.  Use compression-msg:nrobotid instead.")
  (nrobotid m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <msg_features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compression-msg:data-val is deprecated.  Use compression-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msg_features>) ostream)
  "Serializes a message object of type '<msg_features>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tframe))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'nrobotid)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msg_features>) istream)
  "Deserializes a message object of type '<msg_features>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tframe) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nrobotid) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msg_features>)))
  "Returns string type for a message object of type '<msg_features>"
  "compression/msg_features")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msg_features)))
  "Returns string type for a message object of type 'msg_features"
  "compression/msg_features")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msg_features>)))
  "Returns md5sum for a message object of type '<msg_features>"
  "1f479cc2f4feb5562e889083a5793916")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msg_features)))
  "Returns md5sum for a message object of type 'msg_features"
  "1f479cc2f4feb5562e889083a5793916")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msg_features>)))
  "Returns full string definition for message of type '<msg_features>"
  (cl:format cl:nil "Header header~%float64 tframe~%int16 nrobotid~%byte[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msg_features)))
  "Returns full string definition for message of type 'msg_features"
  (cl:format cl:nil "Header header~%float64 tframe~%int16 nrobotid~%byte[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msg_features>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msg_features>))
  "Converts a ROS message object to a list"
  (cl:list 'msg_features
    (cl:cons ':header (header msg))
    (cl:cons ':tframe (tframe msg))
    (cl:cons ':nrobotid (nrobotid msg))
    (cl:cons ':data (data msg))
))
