
(cl:in-package :asdf)

(defsystem "msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Home" :depends-on ("_package_Home"))
    (:file "_package_Home" :depends-on ("_package"))
    (:file "HomeAndLimit" :depends-on ("_package_HomeAndLimit"))
    (:file "_package_HomeAndLimit" :depends-on ("_package"))
  ))