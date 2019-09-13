
(cl:in-package :asdf)

(defsystem "simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Parameters" :depends-on ("_package_Parameters"))
    (:file "_package_Parameters" :depends-on ("_package"))
    (:file "Laser_values" :depends-on ("_package_Laser_values"))
    (:file "_package_Laser_values" :depends-on ("_package"))
  ))