;; Auto-generated. Do not edit!


(when (boundp 'path_tracking::Waypoint)
  (if (not (find-package "PATH_TRACKING"))
    (make-package "PATH_TRACKING"))
  (shadow 'Waypoint (find-package "PATH_TRACKING")))
(unless (find-package "PATH_TRACKING::WAYPOINT")
  (make-package "PATH_TRACKING::WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Waypoint.msg.html


(defclass path_tracking::Waypoint
  :super ros::object
  :slots (_cnt _x_arr _y_arr ))

(defmethod path_tracking::Waypoint
  (:init
   (&key
    ((:cnt __cnt) 0)
    ((:x_arr __x_arr) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y_arr __y_arr) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _cnt (round __cnt))
   (setq _x_arr __x_arr)
   (setq _y_arr __y_arr)
   self)
  (:cnt
   (&optional __cnt)
   (if __cnt (setq _cnt __cnt)) _cnt)
  (:x_arr
   (&optional __x_arr)
   (if __x_arr (setq _x_arr __x_arr)) _x_arr)
  (:y_arr
   (&optional __y_arr)
   (if __y_arr (setq _y_arr __y_arr)) _y_arr)
  (:serialization-length
   ()
   (+
    ;; int32 _cnt
    4
    ;; float64[] _x_arr
    (* 8    (length _x_arr)) 4
    ;; float64[] _y_arr
    (* 8    (length _y_arr)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _cnt
       (write-long _cnt s)
     ;; float64[] _x_arr
     (write-long (length _x_arr) s)
     (dotimes (i (length _x_arr))
       (sys::poke (elt _x_arr i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _y_arr
     (write-long (length _y_arr) s)
     (dotimes (i (length _y_arr))
       (sys::poke (elt _y_arr i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _cnt
     (setq _cnt (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _x_arr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x_arr (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x_arr i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _y_arr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y_arr (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y_arr i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get path_tracking::Waypoint :md5sum-) "b2ba581b0127fa88549ceec9fea43516")
(setf (get path_tracking::Waypoint :datatype-) "path_tracking/Waypoint")
(setf (get path_tracking::Waypoint :definition-)
      "int32 cnt
float64[] x_arr
float64[] y_arr
")



(provide :path_tracking/Waypoint "b2ba581b0127fa88549ceec9fea43516")


