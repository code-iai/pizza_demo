;;;
;;; Copyright (c) 2016, Mihai Pomarlan <blandc@cs.uni-bremen.com>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :pizza-ninja)

(defparameter *tf-listener* nil)

(defparameter *pub-mrk* nil)

(defparameter *base-link* "map")

(defparameter *identity-pose* nil)
(defparameter *identity-pose-msg* nil)

(defparameter *segment-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.2 :g 0.4 :b 0.9))
(defparameter *first-segment-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.2 :g 0.9 :b 0.9))

(defparameter *plate-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.8 :g 0.8 :b 0.8))

(defparameter *pizza-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.8 :g 0.7 :b 0.3))

(defparameter *object-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.8 :g 0.7 :b 0.3))

(defparameter *slice-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.3 :g 0.8 :b 0.3))

(defparameter *cut-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.8 :g 0.3 :b 0.3))

(defparameter *visualization-topic* "/visualization_marker")

(defun ensure-mrk-publisher ()
  (if *pub-mrk*
    *pub-mrk*
    (progn
      (setf *pub-mrk* (roslisp:advertise *visualization-topic* "visualization_msgs/Marker" :latch nil))
      (roslisp:wait-duration 2.0)
      *pub-mrk*)))

(defun destroy-mrk-publisher ()
  (setf *pub-mrk* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-mrk-publisher)


(defun tr->ps (transform)
  (let* ((v (cl-transforms:translation transform))
         (r (cl-transforms:rotation transform))
         (x (cl-transforms:x v))
         (y (cl-transforms:y v))
         (z (cl-transforms:z v))
         (qx (cl-transforms:x r))
         (qy (cl-transforms:y r))
         (qz (cl-transforms:z r))
         (qw (cl-transforms:w r)))
    (roslisp:make-message "geometry_msgs/Pose"
      :position (roslisp:make-message "geometry_msgs/Point" :x x :y y :z z)
      :orientation (roslisp:make-message "geometry_msgs/Quaternion" :x qx :y qy :z qz :w qw))))

(defun make-mrk-msg (base-frame-name frame-locked id action pose color mesh-resource &key (alpha 1) (namespace "cutplan"))
  (roslisp:with-fields (a r g b) color
    (let* ((color (roslisp:make-message "std_msgs/ColorRGBA" :a (* a alpha) :r r :g g :b b)))
      (roslisp:make-message "visualization_msgs/Marker"
                            :header (roslisp:make-message "std_msgs/Header" 
                                                          :frame_id base-frame-name :stamp 0)
                                                          :ns namespace
                                                          :id id
                                                          :frame_locked frame-locked
                                                          :type 10
                                                          :action action
                                                          :pose pose
                                                          :scale (roslisp:make-message "geometry_msgs/Vector3"
                                                                                       :x 1 :y 1 :z 1)
                                                          :color color
                                                          :mesh_resource mesh-resource))))

(defun place-plate-group-markers (where from-pizza-to-plate id-plate id-pizza id-slice base-frame &key (alpha 1) (linked-frame nil) (base-to-link-transform nil))
  (declare (ignore id-plate))
  (let* ((linked-frame (if base-to-link-transform linked-frame nil))
         (where (if linked-frame 
                  (cl-transforms-stamped:transform* (cl-transforms-stamped:transform-inv base-to-link-transform) where)
                  where))
         (pizza-pose-msg (tr->ps where))
         (slice-pose-msg (tr->ps (cl-transforms-stamped:transform* where (cl-transforms:transform-inv from-pizza-to-plate))))
         (base-frame (if linked-frame linked-frame base-frame))
         (frame-locked (if linked-frame 1 0))
         (pizza-msg (make-mrk-msg base-frame frame-locked id-pizza 0 pizza-pose-msg *pizza-color* "package://pizza_demo/models/pizza_plate/meshes/pizza_plate_visual.stl" :alpha alpha))
         (slice-msg (make-mrk-msg base-frame frame-locked id-slice 0 slice-pose-msg *slice-color* "package://pizza_demo/models/pizza_plate/meshes/slice.stl" :alpha alpha)))
    (roslisp:publish (ensure-mrk-publisher) pizza-msg)
    (roslisp:publish (ensure-mrk-publisher) slice-msg)))

(defun place-skeleton-markers (cut-skeleton-wrapper base-frame &key (linked-frame nil) (base-to-link-transform nil))
  (let* ((where (plan-to-environment-transform cut-skeleton-wrapper))
         (linked-frame (if base-to-link-transform linked-frame nil))
         (where (if linked-frame 
                  (cl-transforms-stamped:transform* (cl-transforms-stamped:transform-inv base-to-link-transform) where)
                  where))
         (base-frame (if linked-frame linked-frame base-frame))
         (frame-locked (if linked-frame 1 0))
         (segments (get-segments-for-visualization cut-skeleton-wrapper where))
         (points (apply #'append segments))
         (first-segment (when (and (first points) (second points))
                          (list (first points) (second points))))
         (points (cdr (cdr points)))
         (first-seg-msg (roslisp:make-message "visualization_msgs/Marker"
                                              :header (roslisp:make-message "std_msgs/Header" :frame_id base-frame :stamp 0)
                                              :ns "cut-skeleton"
                                              :id 0
                                              :frame_locked frame-locked
                                              :action 0
                                              :type 0
                                              :scale (roslisp:make-message "geometry_msgs/Vector3"
                                                                           :x 0.01 :y 0.015 :z 0.015)
                                              :points (coerce first-segment 'vector)
                                              :pose (tr->ps (cl-transforms:make-identity-transform))
                                              :color *first-segment-color*))
         ;;(second-seg-msg (roslisp:make-message "visualization_msgs/Marker"
         ;;                                      :header (roslisp:make-message "std_msgs/Header" :frame_id base-frame :stamp 0)
         ;;                                      :ns "cut-skeleton"
         ;;                                      :id 1
         ;;                                      :frame_locked frame-locked
         ;;                                      :action 0
         ;;                                      :type 5
         ;;                                      :points (coerce points 'vector)
         ;;                                      :scale (roslisp:make-message "geometry_msgs/Vector3"
         ;;                                                                   :x 0.01 :y 0.015 :z 0.015)
         ;;                                      :pose (tr->ps (cl-transforms:make-identity-transform))
         ;;                                      :color *segment-color*))
         (ids (alexandria:iota (- (/ (length points) 2) 1) :start 1))
         (starts (select-every points 0 2))
         (ends (select-every points 1 2))
         (next-seg-list (mapcar (lambda (start end id)
                                  (roslisp:make-message "visualization_msgs/Marker"
                                                        :header (roslisp:make-message "std_msgs/Header" :frame_id base-frame :stamp 0)
                                                        :ns "cut-skeleton"
                                                        :id id
                                                        :frame_locked frame-locked
                                                        :action 0
                                                        :type 0
                                                        :points (vector start end)
                                                        :scale (roslisp:make-message "geometry_msgs/Vector3"
                                                                                     :x 0.01 :y 0.015 :z 0.015)
                                                        :pose (tr->ps (cl-transforms:make-identity-transform))
                                                        :color *segment-color*))
                                starts ends ids)))
    (roslisp:publish (ensure-mrk-publisher) first-seg-msg)
    (mapcar (lambda (msg)
              (roslisp:publish (ensure-mrk-publisher) msg))
            next-seg-list)))

(defun get-heatmap-color (value)
  (let* ((blue 0)
         (red (if (< value 0.5) value 0.5))
         (red (* red 2))
         (aux (- value 0.5))
         (green (if (< value 0.5) 0 (* aux aux 4)))
         (green (* green 2)))
    (roslisp:make-message "std_msgs/ColorRGBA" :a 0.25 :r red :g green :b blue)))

(defun place-reachmap-markers (reachmap-slice arm-base-frame obj-position)
  (let* ((slice-points (mapcar (lambda (arg)
                                 (let* ((arg (first arg))
                                        (arg (if (typep arg 'cl-transforms:transform)
                                               (cl-transforms:translation arg)
                                               arg)))
                                   arg))
                               reachmap-slice))
         (slice-ids (alexandria:iota (length reachmap-slice)))
         (slice-rm-scores (mapcar #'second
                                  reachmap-slice))
         (slice-colors (mapcar (lambda (arg)
                                 (get-heatmap-color arg))
                               slice-rm-scores))
         (obj-position (if (typep obj-position 'cl-transforms:transform)
                         (cl-transforms:translation obj-position)
                         obj-position))
         (slice-scores (mapcar (lambda (point score)
                                 (let* ((dist (cl-transforms:v-dist point obj-position)))
                                   (* (/ score (+ (/ (* dist dist) (* 0.2 0.2)) 1)) 0.25)))
                               slice-points slice-rm-scores))
         (reachmap-msgs
           (mapcar (lambda (point score color id)
                     (roslisp:make-message "visualization_msgs/Marker"
                                           :header (roslisp:make-message "std_msgs/Header" :frame_id arm-base-frame :stamp 0)
                                           :ns "reachmap-slice"
                                           :id id
                                           :frame_locked nil
                                           :action 0
                                           :type 3
                                           :pose (tr->ps (cl-transforms:make-transform (cl-transforms:v+ point (cl-transforms:make-3d-vector 0 0 (/ score 2))) (cl-transforms:make-quaternion 0 0 0 1)))
                                           :scale (roslisp:make-message "geometry_msgs/Vector3"
                                                                        :x 0.01 :y 0.01 :z score)
                                           :color color))
                   slice-points slice-scores slice-colors slice-ids))
         (reachmap-msg (roslisp:make-message "visualization_msgs/Marker"
                                             :header (roslisp:make-message "std_msgs/Header" :frame_id arm-base-frame :stamp 0)
                                             :ns "reachmap-slice"
                                             :id 100
                                             :frame_locked nil
                                             :action 0
                                             :type 8
                                             :scale (roslisp:make-message "geometry_msgs/Vector3" :x 0.01 :y 0.01 :z 0.01)
                                             :pose (tr->ps (cl-transforms:make-identity-transform))
                                             :points (coerce (mapcar (lambda (point)
                                                                       (roslisp:make-message "geometry_msgs/Point"
                                                                                             :x (cl-transforms:x point)
                                                                                             :y (cl-transforms:y point)
                                                                                             :z (cl-transforms:z point)))
                                                                     slice-points)
                                                             'vector)
                                             :colors (coerce slice-colors 'vector))))
    (mapcar (lambda (reachmap-msg) (roslisp:publish (ensure-mrk-publisher) reachmap-msg)) (subseq reachmap-msgs 0 100))
    (roslisp:publish (ensure-mrk-publisher) reachmap-msg)
    ))

(defun place-reachmap-slice-markers (reachability-map z arm-base-transform threshold)
  (let* ((reachmap-slice (get-reachmap-slice reachability-map z arm-base-transform threshold))
         (scored-points (mapcar (lambda (arg)
                                  (let* ((p (first arg)))
                                    (list p (second arg))))
                                reachmap-slice))
         (slice-points (mapcar #'first scored-points))
         (slice-points (mapcar #'cl-transforms:translation slice-points))
         (slice-scores (mapcar #'second scored-points))
         (slice-colors (mapcar (lambda (arg)
                                 (get-heatmap-color arg))
                               slice-scores))
         (reachmap-msg (roslisp:make-message "visualization_msgs/Marker"
                                             :header (roslisp:make-message "std_msgs/Header" :frame_id "map" :stamp 0)
                                             :ns "reachmap-dump"
                                             :id 0
                                             :frame_locked nil
                                             :action 0
                                             :type 8
                                             :scale (roslisp:make-message "geometry_msgs/Vector3" :x 0.01 :y 0.01 :z 0.01)
                                             :pose (tr->ps (cl-transforms:make-identity-transform))
                                             :points (coerce (mapcar (lambda (point)
                                                                       (roslisp:make-message "geometry_msgs/Point"
                                                                                             :x (cl-transforms:x point)
                                                                                             :y (cl-transforms:y point)
                                                                                             :z (cl-transforms:z point)))
                                                                     slice-points)
                                                             'vector)
                                             :colors (coerce slice-colors 'vector))))
    (roslisp:publish (ensure-mrk-publisher) reachmap-msg)))

(defun get-reachmap-markers (reachability-map arm-base-transform threshold &key (fixed-frame "map"))
  (let* ((scored-points (get-reachmap-scored-points reachability-map arm-base-transform threshold))
         (slice-points (mapcar #'first scored-points))
         (slice-points (mapcar #'cl-transforms:translation slice-points))
         (slice-scores (mapcar #'second scored-points))
         (slice-colors (mapcar (lambda (arg)
                                 (get-heatmap-color arg))
                               slice-scores))
         (reachmap-msg (roslisp:make-message "visualization_msgs/Marker"
                                             :header (roslisp:make-message "std_msgs/Header" :frame_id fixed-frame :stamp 0)
                                             :ns "reachmap-dump"
                                             :id 0
                                             :frame_locked nil
                                             :action 0
                                             :type 8
                                             :scale (roslisp:make-message "geometry_msgs/Vector3" :x 0.01 :y 0.01 :z 0.01)
                                             :pose (tr->ps (cl-transforms:make-identity-transform))
                                             :points (coerce (mapcar (lambda (point)
                                                                       (roslisp:make-message "geometry_msgs/Point"
                                                                                             :x (cl-transforms:x point)
                                                                                             :y (cl-transforms:y point)
                                                                                             :z (cl-transforms:z point)))
                                                                     slice-points)
                                                             'vector)
                                             :colors (coerce slice-colors 'vector))))
    (roslisp:publish (ensure-mrk-publisher) reachmap-msg)))

(defun clear-reachmap-markers (base-frame)
  (let* ((reachmap-msg (roslisp:make-message "visualization_msgs/Marker"
                                             :header (roslisp:make-message "std_msgs/Header" :frame_id base-frame :stamp 0)
                                             :ns "reachmap-slice"
                                             :id 0
                                             :action 3)))
    (roslisp:publish (ensure-mrk-publisher) reachmap-msg)))
