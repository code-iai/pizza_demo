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

(defparameter *pub-mrk* nil)
(defparameter *RVIZ-ADD-MARKER* 0)
(defparameter *RVIZ-DEL-MARKER* 2)
(defparameter *RVIZ-DEL-ALL-MARKER* 3)

(defparameter *segment-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.2 :g 0.4 :b 0.9))
(defparameter *first-segment-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.2 :g 0.9 :b 0.9))
(defparameter *object-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.8 :g 0.7 :b 0.3))
(defparameter *slice-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.3 :g 0.8 :b 0.3))

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

(defun get-heatmap-color (value)
  (let* ((blue 0)
         (red (if (< value 0.5) value 0.5))
         (red (* red 2))
         (aux (- value 0.5))
         (green (if (< value 0.5) 0 (* aux aux 4)))
         (green (* green 2)))
    (roslisp:make-message "std_msgs/ColorRGBA" :a 0.25 :r red :g green :b blue)))

(defun make-mrk-msg (base-frame-name &key
                     (pose (tr->ps (cl-transforms:make-identity-transform))) (action *RVIZ-ADD-MARKER*) (id 0) (type 0) (color *object-color*) (frame-locked 0) (colors (coerce nil 'vector))
                     (mesh-resource "") (alpha 1) (namespace "cutplan") (scale (roslisp:make-message "geometry_msgs/Vector3" :x 1 :y 1 :z 1)) (points (coerce nil 'vector)))
  (roslisp:with-fields (a r g b) color
    (let* ((color (roslisp:make-message "std_msgs/ColorRGBA" :a (* a alpha) :r r :g g :b b)))
      (roslisp:make-message "visualization_msgs/Marker"
                            :header (roslisp:make-message "std_msgs/Header" :frame_id base-frame-name :stamp 0)
                            :ns namespace
                            :id id
                            :frame_locked frame-locked
                            :action action
                            :type type
                            :pose pose
                            :scale scale
                            :color color
                            :color colors
                            :points points
                            :mesh_resource mesh-resource))))

(defun make-mesh-marker-msg (base-frame-name mesh-resource &key
                             (color *object-color*) (frame-locked 0) (id 0) (action 0) (pose (tr->ps (cl-transforms:make-identity-transform))) (alpha 1) (namespace "cutplan"))
  (make-mrk-msg base-frame-name :frame-locked frame-locked :color color :id id :action action :pose pose :type 10 :alpha alpha :namespace namespace :mesh-resource mesh-resource))

(defun place-skeleton-markers (cut-skeleton-wrapper base-frame &key (linked-frame nil) (base-to-link-transform nil))
  (let* ((where (plan-to-environment-transform cut-skeleton-wrapper))
         (linked-frame (if base-to-link-transform linked-frame nil))
         (where (if linked-frame 
                  (cl-transforms-stamped:transform* (cl-transforms-stamped:transform-inv base-to-link-transform) where)
                  where))
         (base-frame (if linked-frame linked-frame base-frame))
         (frame-locked (if linked-frame 1 0))
         (arrow-scale (roslisp:make-message "geometry_msgs/Vector3" :x 0.01 :y 0.015 :z 0.015))
         (segments (get-segments-for-visualization cut-skeleton-wrapper where))
         (first-segment (when (car segments)
                          (car segments)))
         (next-segments (cdr segments))
         (first-seg-msg (when first-segment
                          (make-mrk-msg base-frame
                                        :frame-locked frame-locked :color *first-segment-color* :namespace "cut-skeleton" 
                                        :scale arrow-scale
                                        :points (coerce first-segment 'vector))))
         (ids (alexandria:iota (length next-segments) :start 1))
         (next-seg-list (mapcar (lambda (segment id)
                                  (make-mrk-msg base-frame
                                                :frame-locked frame-locked :color *segment-color* :namespace "cut-skeleton" :id id
                                                :scale arrow-scale
                                                :points (coerce segment 'vector)))
                                next-segments ids)))
    (mapcar (lambda (id) 
              (roslisp:publish (ensure-mrk-publisher)
                               (make-mrk-msg base-frame :namespace "cut-skeleton" :action *RVIZ-DEL-MARKER* :id id :scale (roslisp:make-message "geometry_msgs/Vector3" :x 0.001 :y 0.001 :z 0.001))))
            (alexandria:iota 10))
    (when first-seg-msg
      (roslisp:publish (ensure-mrk-publisher) first-seg-msg))
    (mapcar (lambda (msg)
              (roslisp:publish (ensure-mrk-publisher) msg))
            next-seg-list)))

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
                     (make-mrk-msg arm-base-frame :namespace "reachmap-slice" :id id :type 3 :color color
                                   :pose (tr->ps (cl-transforms:make-transform (cl-transforms:v+ point (cl-transforms:make-3d-vector 0 0 (/ score 2))) (cl-transforms:euler->quaternion)))
                                   :scale (roslisp:make-message "geometry_msgs/Vector3" :x 0.01 :y 0.01 :z score)))
                   slice-points slice-scores slice-colors slice-ids))
         (reachmap-msg (make-mrk-msg arm-base-frame :namespace "reachmap-slice-sphere" :id 100 :type 8 :colors (coerce slice-colors 'vector)
                                     :pose (tr->ps (cl-transforms:make-identity-transform))
                                     :scale (roslisp:make-message "geometry_msgs/Vector3" :x 0.01 :y 0.01 :z 0.01)
                                     :points (coerce (mapcar (lambda (point)
                                                               (roslisp:make-message "geometry_msgs/Point"
                                                                                     :x (cl-transforms:x point)
                                                                                     :y (cl-transforms:y point)
                                                                                     :z (cl-transforms:z point)))
                                                             slice-points)
                                                     'vector))))
    (mapcar (lambda (reachmap-msg) 
              (roslisp:publish (ensure-mrk-publisher) reachmap-msg)) (subseq reachmap-msgs 0 100))
    (roslisp:publish (ensure-mrk-publisher) reachmap-msg)))

