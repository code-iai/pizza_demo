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

(defclass skeleton-segment ()
  ((segment-start :initform nil :initarg :segment-start :reader segment-start)
   (segment-end :initform nil :initarg :segment-end :reader segment-end)
   (segment-prestart :initform nil :initarg :segment-prestart :reader segment-prestart)
   (segment-postend :initform nil :initarg :segment-postend :reader segment-postend)))

(defclass cut-skeleton-wrapper ()
  ((cut-skeleton :initform nil :initarg :cut-skeleton :accessor cut-skeleton)
   (skeleton-to-tool-transform :initform nil :initarg :skeleton-to-tool-transform :accessor skeleton-to-tool-transform)
   (plan-to-environment-transform :initform nil :initarg :plan-to-environment-transform :accessor plan-to-environment-transform)))

(defparameter *prep-offset* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0 0 0.1)
                                                          (cl-transforms:euler->quaternion)))


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

(defun vec->pnt (vec)
  (roslisp:make-message "geometry_msgs/Point"
                        :x (cl-transforms:x vec)
                        :y (cl-transforms:y vec)
                        :z (cl-transforms:z vec)))

(defun select-every (list idx mod)
  (let* ((k 0))
    (remove-if #'not
               (mapcar (lambda (x)
                         (setf k (+ k 1))
                         (if (equal (mod (- k 1) mod) idx)
                           x
                           nil))
                       list))))

(defun interpret-maneuver-parameters (maneuver-type maneuver-parameters)
  (cond
    ((equal maneuver-type "cut")
      (let* ((maneuver-parameters (coerce maneuver-parameters 'list))
             (xs (select-every maneuver-parameters 0 6))
             (ys (select-every maneuver-parameters 1 6))
             (zs (select-every maneuver-parameters 2 6))
             (xe (select-every maneuver-parameters 3 6))
             (ye (select-every maneuver-parameters 4 6))
             (ze (select-every maneuver-parameters 5 6))
             (starts (mapcar #'cl-transforms:make-3d-vector xs ys zs))
             (ends (mapcar #'cl-transforms:make-3d-vector xe ye ze))
             (dirs-along (mapcar (alexandria:compose #'cl-transforms:normalize-vector #'cl-transforms:v-)
                                 ends
                                 starts))
             (dirs-vertical (mapcar (lambda (dirx)
                                      (let* ((vert (cl-transforms:make-3d-vector 0 0 1))
                                             (along (cl-transforms:dot-product vert dirx))
                                             (along (cl-transforms:v* dirx along))
                                             (vert (cl-transforms:v- vert along)))
                                        (cl-transforms:normalize-vector vert)))
                                    dirs-along))
             (dirs-side (mapcar #'cl-transforms:cross-product dirs-vertical dirs-along)))
        (mapcar (lambda (start end dir-x dir-y dir-z)
                  (let* ((orientation (cl-transforms:column-vectors->quaternion dir-x dir-y dir-z))
                         (segment-start (cl-transforms:make-transform start orientation))
                         (segment-end (cl-transforms:make-transform end orientation)))
                    (make-instance 'skeleton-segment
                                   :segment-start segment-start
                                   :segment-end segment-end)))
                starts
                ends
                dirs-along
                dirs-side
                dirs-vertical)))
    (T
      (error "Unrecognized maneuver type ~a." maneuver-type))))

(defun vector-pair->skeleton-segment (start end)
  (let* ((xs (cl-transforms:x start))
         (ys (cl-transforms:y start))
         (zs (cl-transforms:z start))
         (xe (cl-transforms:x end))
         (ye (cl-transforms:y end))
         (ze (cl-transforms:z end))
         (maneuver-parameters (list xs ys zs xe ye ze)))
    (car (interpret-maneuver-parameters "cut" maneuver-parameters))))

(defun flip-segment (skeleton-segment)
  (let* ((start (cl-transforms-stamped:translation (segment-start skeleton-segment)))
         (end (cl-transforms-stamped:translation (segment-end skeleton-segment))))
    (roslisp:with-fields ((xs x) (ys y) (zs z)) start
      (roslisp:with-fields ((xe x) (ye y) (ze z)) end
        (first (interpret-maneuver-parameters "cut" (list xe ye ze xs ys zs)))))))

(defun segment-angle (skeleton-segment arm-base-transform)
  (let* ((xy-plane-pose (cl-transforms:make-transform (cl-transforms:make-3d-vector (cl-transforms:x (cl-transforms:translation arm-base-transform))
                                                                                    (cl-transforms:y (cl-transforms:translation arm-base-transform))
                                                                                    0)
                                                      (cl-transforms:rotation arm-base-transform)))
         (xy-plane-pose (cl-transforms:transform-inv xy-plane-pose))
         (skeleton-segment (make-instance 'skeleton-segment
                                          :segment-start (cl-transforms:transform* xy-plane-pose
                                                                                   (cl-transforms:make-transform
                                                                                     (cl-transforms:translation (segment-start skeleton-segment))
                                                                                     (cl-transforms:euler->quaternion)))
                                          :segment-end (cl-transforms:transform* xy-plane-pose
                                                                                 (cl-transforms:make-transform
                                                                                   (cl-transforms:translation (segment-end skeleton-segment))
                                                                                   (cl-transforms:euler->quaternion)))))
         (start (cl-transforms-stamped:translation (segment-start skeleton-segment)))
         (end (cl-transforms-stamped:translation (segment-end skeleton-segment))))
    (roslisp:with-fields ((xs x) (ys y) (zs z)) start
      (declare (ignore zs))
      (roslisp:with-fields ((xe x) (ye y) (ze z)) end
        (declare (ignore ze))
        (let* ((x (- xe xs))
               (y (- ye ys))
               (n (+ (* x x) (* y y))))
          (if (> n 0.000001)
            (atan y x)
            0))))))

(defun transform-segment-point (pl-to-en sk-to-tl point &key (prep-offset (cl-transforms:make-identity-transform)))
  (cl-transforms:transform* pl-to-en point prep-offset sk-to-tl))

(defun transform-skeleton-segment (skeleton-segment sk-to-tl pl-to-en prep-offset)
  (let* ((segment-start (segment-start skeleton-segment))
         (segment-end (segment-end skeleton-segment)))
    (make-instance 'skeleton-segment
                   :segment-start (transform-segment-point pl-to-en sk-to-tl segment-start)
                   :segment-end (transform-segment-point pl-to-en sk-to-tl segment-end)
                   :segment-prestart (transform-segment-point pl-to-en sk-to-tl segment-start :prep-offset prep-offset)
                   :segment-postend (transform-segment-point pl-to-en sk-to-tl segment-end :prep-offset prep-offset))))

(defun create-cut-skeleton-wrapper (maneuver-parameters skeleton-to-tool-transform plan-to-environment-transform)
  (make-instance 'cut-skeleton-wrapper
                 :cut-skeleton (interpret-maneuver-parameters "cut" maneuver-parameters)
                 :skeleton-to-tool-transform skeleton-to-tool-transform
                 :plan-to-environment-transform plan-to-environment-transform))

(defun get-current-segment (cut-skeleton-wrapper &optional (prep-offset *prep-offset*))
  (transform-skeleton-segment (car (cut-skeleton cut-skeleton-wrapper)) 
                              (skeleton-to-tool-transform cut-skeleton-wrapper)
                              (plan-to-environment-transform cut-skeleton-wrapper)
                              prep-offset))

(defun get-segments-for-visualization (cut-skeleton-wrapper plan-to-environment-transform)
  (mapcar (lambda (segment)
            (let* ((segment (transform-skeleton-segment segment
                                                        *prep-offset* 
                                                        plan-to-environment-transform
                                                        *prep-offset*))
                   (segment-start (cl-transforms:translation (segment-start segment)))
                   (segment-end (cl-transforms:translation (segment-end segment)))
                   (segment-start (vec->pnt segment-start))
                   (segment-end (vec->pnt segment-end)))
              (list segment-start segment-end)))
          (cut-skeleton cut-skeleton-wrapper)))

(defun pop-skeleton-segment (cut-skeleton-wrapper)
  (setf (cut-skeleton cut-skeleton-wrapper) (cdr (cut-skeleton cut-skeleton-wrapper))))

(defun suggest-placement-orientation (cut-skeleton-wrapper maneuver-arm arm-base-transform)
  (let* ((maneuver-arm (if (or (equal maneuver-arm :left) (equal maneuver-arm :right))
                         (if (equal maneuver-arm :left)
                           "left"
                           "right")
                         (string-downcase maneuver-arm)))
         (p-e-tran (cl-transforms-stamped:make-transform (cl-transforms-stamped:translation (plan-to-environment-transform cut-skeleton-wrapper))
                                                         (cl-transforms-stamped:make-quaternion 0 0 0 1)))
         ;;;;(current-segment (get-current-segment cut-skeleton-wrapper))
         ;;;;(segment-angle (segment-angle current-segment arm-base-transform))
         ;;(dummy (format t "SEG-ANGLE ~a~%" segment-angle))
         (quarter-pi (atan 1))
         (pref-angle (if (equal maneuver-arm "left")
                         (- 0 quarter-pi)
                         quarter-pi))
         (max-angle (+ pref-angle (* 2 quarter-pi)))
         (min-angle (- pref-angle (* 2 quarter-pi)))
         ;;;; !!!!
         ;;(dummy (if (or (< max-angle segment-angle) (< segment-angle min-angle))
         ;;         (setf (car (cut-skeleton cut-skeleton-wrapper)) (flip-segment (car (cut-skeleton cut-skeleton-wrapper))))))
         (current-segment (get-current-segment cut-skeleton-wrapper))
         (segment-angle (segment-angle current-segment arm-base-transform))
         (rot-angle (- pref-angle segment-angle))
         (ortran (cl-transforms-stamped:make-transform (cl-transforms-stamped:make-3d-vector 0 0 0)
                                                       (cl-transforms-stamped:axis-angle->quaternion (vector 0.0 0.0 1.0) rot-angle))))
    ;; (declare (ignore dummy))
    (declare (ignore max-angle) (ignore min-angle))
    (cl-transforms-stamped:transform* p-e-tran ortran (cl-transforms-stamped:transform-inv p-e-tran))))

(defun segment-reachable (orientation-transform segment reachability-map)
  (if reachability-map
    (let* ((segment-start (cl-transforms-stamped:transform* orientation-transform (segment-start segment)))
           (segment-end (cl-transforms-stamped:transform* orientation-transform (segment-end segment)))
           (segment-prestart (cl-transforms-stamped:transform* orientation-transform (segment-prestart segment)))
           (segment-postend (cl-transforms-stamped:transform* orientation-transform (segment-postend segment)))
           (segment-start (pr2-reachability-costmap:pose-reachable-p reachability-map (cl-transforms-stamped:transform->pose segment-start) :use-closest-orientation T))
           (segment-end (pr2-reachability-costmap:pose-reachable-p reachability-map (cl-transforms-stamped:transform->pose segment-end) :use-closest-orientation T))
           (segment-prestart (pr2-reachability-costmap:pose-reachable-p reachability-map (cl-transforms-stamped:transform->pose segment-prestart) :use-closest-orientation T))
           (segment-postend (pr2-reachability-costmap:pose-reachable-p reachability-map (cl-transforms-stamped:transform->pose segment-postend) :use-closest-orientation T)))
      (and segment-start segment-end segment-prestart segment-postend))
    T))

(defun get-reachmap-slice (reachability-map z arm-base-transform threshold)
  (let* ((z (- z (cl-transforms-stamped:z (cl-transforms:translation arm-base-transform))))
         (map-x-origin (cl-transforms-stamped:x (pr2-reachability-costmap::origin reachability-map)))
         (map-y-origin (cl-transforms-stamped:y (pr2-reachability-costmap::origin reachability-map)))
         (map-z-origin (cl-transforms-stamped:z (pr2-reachability-costmap::origin reachability-map)))
         (map-x-resolution (cl-transforms-stamped:x (pr2-reachability-costmap::resolution reachability-map)))
         (map-y-resolution (cl-transforms-stamped:y (pr2-reachability-costmap::resolution reachability-map)))
         (map-z-resolution (cl-transforms-stamped:z (pr2-reachability-costmap::resolution reachability-map)))
         (z-index (pr2-reachability-costmap::map-coordinate->array-index z map-z-resolution map-z-origin))
         (matrix (pr2-reachability-costmap::reachability-map reachability-map))
         (x-max (1- (array-dimension matrix 2)))
         (y-max (1- (array-dimension matrix 1)))
         (candidates nil))
    (if (and (>= z-index 0) (> (array-dimension matrix 0) z-index))
      (progn
        (loop for k from 0 upto y-max do
          (loop for j from 0 upto x-max do
            (let* ((candidate (cl-transforms-stamped:make-transform (cl-transforms-stamped:make-3d-vector
                                                                      (pr2-reachability-costmap::array-index->map-coordinate j map-x-resolution map-x-origin)
                                                                      (pr2-reachability-costmap::array-index->map-coordinate k map-y-resolution map-y-origin)
                                                                      (pr2-reachability-costmap::array-index->map-coordinate z-index map-z-resolution map-z-origin))
                                                                    (cl-transforms-stamped:make-identity-rotation)))
                   (score (pr2-reachability-costmap:pose-reachability reachability-map (cl-transforms-stamped:transform->pose candidate)))
                   (candidate (cl-transforms-stamped:transform* arm-base-transform candidate)))
              (if (> score threshold)
                (setf candidates (cons (list candidate score) candidates))))))
        candidates)
      nil)))

(defun suggest-placement-transform (cut-skeleton-wrapper maneuver-arm arm-base-transform &key (reachability-map nil))
  (let* ((orientation-transform (suggest-placement-orientation cut-skeleton-wrapper maneuver-arm arm-base-transform))
         (segment-reachable (segment-reachable orientation-transform (get-current-segment cut-skeleton-wrapper) reachability-map)))
    (if (or segment-reachable (not reachability-map))
      (progn 
        (list (cl-transforms:transform* orientation-transform (plan-to-environment-transform cut-skeleton-wrapper))))
      (let* ((start-segment (cl-transforms-stamped:translation (segment-start (get-current-segment cut-skeleton-wrapper))))
             (start-segment (cl-transforms-stamped:transform-point orientation-transform start-segment))
             (z (cl-transforms-stamped:z start-segment))
             (reachmap-slice (get-reachmap-slice reachability-map z arm-base-transform 0.3))
             (position-transforms (mapcar (lambda (candidate)
                                            (let* ((score (second candidate))
                                                   (candidate (first candidate))
                                                   (candidate (cl-transforms:v- (cl-transforms-stamped:translation candidate) start-segment))
                                                   (candidate (cl-transforms:make-3d-vector (cl-transforms:x candidate) 
                                                                                            (cl-transforms:y candidate)
                                                                                            0))
                                                   (distance (cl-transforms:v-norm candidate))
                                                   (score (/ score (1+ (* distance distance)))))
                                              (if (> score 0.3)
                                                (list (cl-transforms:make-transform candidate (cl-transforms:make-quaternion 0 0 0 1))
                                                      score)
                                                nil)))
                                          reachmap-slice))
             (position-transforms (remove-if #'null position-transforms))
             (position-transforms (mapcar (lambda (scored-position-transform)
                                            (list (cl-transforms:transform* (first scored-position-transform) orientation-transform (plan-to-environment-transform cut-skeleton-wrapper))
                                                  (second scored-position-transform)))
                                          position-transforms))
             (position-transforms (sort position-transforms
                                        (lambda (a b)
                                          (> (second a) (second b))))))
        position-transforms))))

(defun move-skeleton (cut-skeleton-wrapper move-transform)
  (setf (plan-to-environment-transform cut-skeleton-wrapper)
        (cl-transforms-stamped:transform* move-transform (plan-to-environment-transform cut-skeleton-wrapper))))

