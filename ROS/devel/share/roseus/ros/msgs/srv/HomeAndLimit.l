;; Auto-generated. Do not edit!


(when (boundp 'msgs::HomeAndLimit)
  (if (not (find-package "MSGS"))
    (make-package "MSGS"))
  (shadow 'HomeAndLimit (find-package "MSGS")))
(unless (find-package "MSGS::HOMEANDLIMIT")
  (make-package "MSGS::HOMEANDLIMIT"))
(unless (find-package "MSGS::HOMEANDLIMITREQUEST")
  (make-package "MSGS::HOMEANDLIMITREQUEST"))
(unless (find-package "MSGS::HOMEANDLIMITRESPONSE")
  (make-package "MSGS::HOMEANDLIMITRESPONSE"))

(in-package "ROS")





(defclass msgs::HomeAndLimitRequest
  :super ros::object
  :slots (_home_xyz _home_oreintation _limit_range ))

(defmethod msgs::HomeAndLimitRequest
  (:init
   (&key
    ((:home_xyz __home_xyz) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:home_oreintation __home_oreintation) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:limit_range __limit_range) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _home_xyz __home_xyz)
   (setq _home_oreintation __home_oreintation)
   (setq _limit_range __limit_range)
   self)
  (:home_xyz
   (&optional __home_xyz)
   (if __home_xyz (setq _home_xyz __home_xyz)) _home_xyz)
  (:home_oreintation
   (&optional __home_oreintation)
   (if __home_oreintation (setq _home_oreintation __home_oreintation)) _home_oreintation)
  (:limit_range
   (&optional __limit_range)
   (if __limit_range (setq _limit_range __limit_range)) _limit_range)
  (:serialization-length
   ()
   (+
    ;; float64[3] _home_xyz
    (* 8    3)
    ;; float64[4] _home_oreintation
    (* 8    4)
    ;; float64[6] _limit_range
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _home_xyz
     (dotimes (i 3)
       (sys::poke (elt _home_xyz i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _home_oreintation
     (dotimes (i 4)
       (sys::poke (elt _home_oreintation i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _limit_range
     (dotimes (i 6)
       (sys::poke (elt _limit_range i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _home_xyz
   (dotimes (i (length _home_xyz))
     (setf (elt _home_xyz i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _home_oreintation
   (dotimes (i (length _home_oreintation))
     (setf (elt _home_oreintation i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _limit_range
   (dotimes (i (length _limit_range))
     (setf (elt _limit_range i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass msgs::HomeAndLimitResponse
  :super ros::object
  :slots (_done ))

(defmethod msgs::HomeAndLimitResponse
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

(defclass msgs::HomeAndLimit
  :super ros::object
  :slots ())

(setf (get msgs::HomeAndLimit :md5sum-) "37986a4fb94d2bd1e620d61c5fcb7476")
(setf (get msgs::HomeAndLimit :datatype-) "msgs/HomeAndLimit")
(setf (get msgs::HomeAndLimit :request) msgs::HomeAndLimitRequest)
(setf (get msgs::HomeAndLimit :response) msgs::HomeAndLimitResponse)

(defmethod msgs::HomeAndLimitRequest
  (:response () (instance msgs::HomeAndLimitResponse :init)))

(setf (get msgs::HomeAndLimitRequest :md5sum-) "37986a4fb94d2bd1e620d61c5fcb7476")
(setf (get msgs::HomeAndLimitRequest :datatype-) "msgs/HomeAndLimitRequest")
(setf (get msgs::HomeAndLimitRequest :definition-)
      "float64[3] home_xyz
float64[4] home_oreintation
float64[6] limit_range
---
int8 done
")

(setf (get msgs::HomeAndLimitResponse :md5sum-) "37986a4fb94d2bd1e620d61c5fcb7476")
(setf (get msgs::HomeAndLimitResponse :datatype-) "msgs/HomeAndLimitResponse")
(setf (get msgs::HomeAndLimitResponse :definition-)
      "float64[3] home_xyz
float64[4] home_oreintation
float64[6] limit_range
---
int8 done
")



(provide :msgs/HomeAndLimit "37986a4fb94d2bd1e620d61c5fcb7476")


