;; Auto-generated. Do not edit!


(when (boundp 'msgs::Home)
  (if (not (find-package "MSGS"))
    (make-package "MSGS"))
  (shadow 'Home (find-package "MSGS")))
(unless (find-package "MSGS::HOME")
  (make-package "MSGS::HOME"))
(unless (find-package "MSGS::HOMEREQUEST")
  (make-package "MSGS::HOMEREQUEST"))
(unless (find-package "MSGS::HOMERESPONSE")
  (make-package "MSGS::HOMERESPONSE"))

(in-package "ROS")





(defclass msgs::HomeRequest
  :super ros::object
  :slots (_home ))

(defmethod msgs::HomeRequest
  (:init
   (&key
    ((:home __home) 0)
    )
   (send-super :init)
   (setq _home (round __home))
   self)
  (:home
   (&optional __home)
   (if __home (setq _home __home)) _home)
  (:serialization-length
   ()
   (+
    ;; int8 _home
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _home
       (write-byte _home s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _home
     (setq _home (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _home 127) (setq _home (- _home 256)))
   ;;
   self)
  )

(defclass msgs::HomeResponse
  :super ros::object
  :slots (_done ))

(defmethod msgs::HomeResponse
  (:init
   (&key
    ((:done __done) 0)
    )
   (send-super :init)
   (setq _done (round __done))
   self)
  (:done
   (&optional __done)
   (if __done (setq _done __done)) _done)
  (:serialization-length
   ()
   (+
    ;; int8 _done
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _done
       (write-byte _done s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _done
     (setq _done (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _done 127) (setq _done (- _done 256)))
   ;;
   self)
  )

(defclass msgs::Home
  :super ros::object
  :slots ())

(setf (get msgs::Home :md5sum-) "315d885f8f6e90abb232f2881df249a2")
(setf (get msgs::Home :datatype-) "msgs/Home")
(setf (get msgs::Home :request) msgs::HomeRequest)
(setf (get msgs::Home :response) msgs::HomeResponse)

(defmethod msgs::HomeRequest
  (:response () (instance msgs::HomeResponse :init)))

(setf (get msgs::HomeRequest :md5sum-) "315d885f8f6e90abb232f2881df249a2")
(setf (get msgs::HomeRequest :datatype-) "msgs/HomeRequest")
(setf (get msgs::HomeRequest :definition-)
      "int8 home
---
int8 done
")

(setf (get msgs::HomeResponse :md5sum-) "315d885f8f6e90abb232f2881df249a2")
(setf (get msgs::HomeResponse :datatype-) "msgs/HomeResponse")
(setf (get msgs::HomeResponse :definition-)
      "int8 home
---
int8 done
")



(provide :msgs/Home "315d885f8f6e90abb232f2881df249a2")


