; Auto-generated. Do not edit!


(cl:in-package marvin-msg)


;//! \htmlinclude lidar.msg.html

(cl:defclass <lidar> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass lidar (<lidar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lidar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lidar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name marvin-msg:<lidar> is deprecated: use marvin-msg:lidar instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <lidar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marvin-msg:id-val is deprecated.  Use marvin-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <lidar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marvin-msg:data-val is deprecated.  Use marvin-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lidar>) ostream)
  "Serializes a message object of type '<lidar>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'data)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lidar>) istream)
  "Deserializes a message object of type '<lidar>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'data)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lidar>)))
  "Returns string type for a message object of type '<lidar>"
  "marvin/lidar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lidar)))
  "Returns string type for a message object of type 'lidar"
  "marvin/lidar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lidar>)))
  "Returns md5sum for a message object of type '<lidar>"
  "6bdfa24e970e7ffd91c15653cd88be54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lidar)))
  "Returns md5sum for a message object of type 'lidar"
  "6bdfa24e970e7ffd91c15653cd88be54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lidar>)))
  "Returns full string definition for message of type '<lidar>"
  (cl:format cl:nil "uint8 id~%uint64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lidar)))
  "Returns full string definition for message of type 'lidar"
  (cl:format cl:nil "uint8 id~%uint64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lidar>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lidar>))
  "Converts a ROS message object to a list"
  (cl:list 'lidar
    (cl:cons ':id (id msg))
    (cl:cons ':data (data msg))
))
