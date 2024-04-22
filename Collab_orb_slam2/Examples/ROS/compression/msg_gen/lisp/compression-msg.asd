
(cl:in-package :asdf)

(defsystem "compression-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "msg_features" :depends-on ("_package_msg_features"))
    (:file "_package_msg_features" :depends-on ("_package"))
  ))