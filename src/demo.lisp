;;;
;;; Copyright (c) 2016, Mihai Pomarlan <blandc@cs.uni-bremen.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(defun cut-test (&optional (visualization-topic "/visualization_marker"))
  (cutplan:start-cutplan-node)
  (setf *pub-mrk* (roslisp:advertise visualization-topic "visualization_msgs/Marker" :latch nil))
  (roslisp:call-service "/meshproc_csg/UnloadMesh" "meshproc_msgs/UnloadMesh"
                        :mesh_name "pizza")
  (roslisp:call-service "/meshproc_csg/UnloadMesh" "meshproc_msgs/UnloadMesh"
                        :mesh_name "slice")
  (roslisp:call-service "/meshproc_csg/UnloadMesh" "meshproc_msgs/UnloadMesh"
                        :mesh_name "dilated-slice")
  (roslisp:call-service "/meshproc_csg/LoadMesh" "meshproc_msgs/LoadMesh"
                        :mesh_name "pizza"
                        :mesh_filenames (list (format nil "~ameshes/pizza.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
                        :duplicate_dist 0.0001)
  (roslisp:call-service "/meshproc_csg/LoadMesh" "meshproc_msgs/LoadMesh"
                        :mesh_name "slice"
                        :mesh_filenames (list (format nil "~ameshes/slice.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
                        :duplicate_dist 0.0001)
  (roslisp:call-service "/meshproc_csg/LoadMesh" "meshproc_msgs/LoadMesh"
                        :mesh_name "dilated-slice"
                        :mesh_filenames (list (format nil "~ameshes/dilated-slice.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
                        :duplicate_dist 0.0001)
  (let* ((result (roslisp:call-service "/cutplan/GetManeuver" 'cutplan-srv:GetManeuver
                                       :maneuver "cut"
                                       :object "pizza"
                                       :target "slice"
                                       :dilated_target "dilated-slice"
                                       :forbidden ""
                                       :maneuver_mesh "cut-mesh"
                                       :new_target "pizza-slice"
                                       :new_object "pizza-remainder")))
    (roslisp:wait-duration 1.0)
    (setf *identity-pose* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.0 0.0 0.0)
                                                      (cl-transforms:make-quaternion 0 0 0 1)))
    (roslisp:call-service "/meshproc_csg/GetMesh" "meshproc_msgs/GetMesh"
                          :mesh_name "pizza-remainder"
                          :return_result 0
                          :result_to_file 1
                          :result_filename (format nil "~a/outputs/new-pizza.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
    (roslisp:call-service "/meshproc_csg/GetMesh" "meshproc_msgs/GetMesh"
                          :mesh_name "pizza-slice"
                          :return_result 0
                          :result_to_file 1
                          :result_filename (format nil "~a/outputs/new-slice.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
    (roslisp:call-service "/meshproc_csg/GetMesh" "meshproc_msgs/GetMesh"
                          :mesh_name "cut-mesh"
                          :return_result 0
                          :result_to_file 1
                          :result_filename (format nil "~a/outputs/new-cut.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
    (setf *identity-pose-msg* (tr->ps *identity-pose*))
    (roslisp:publish *pub-mrk* (make-mrk-msg *base-link* 0 1 0 *identity-pose-msg* *pizza-color* "package://cutplan/outputs/new-pizza.stl"))
    (roslisp:publish *pub-mrk* (make-mrk-msg *base-link* 0 2 0 *identity-pose-msg* *slice-color* "package://cutplan/outputs/new-slice.stl"))
    (roslisp:publish *pub-mrk* (make-mrk-msg *base-link* 0 3 0 *identity-pose-msg* *cut-color* "package://cutplan/outputs/new-cut.stl"))
    (roslisp:with-fields (parameters) result
      (coerce parameters 'list))))



(defun move-arm-pose (arm pose)
  (pr2-manip-pm::execute-move-arm-pose arm pose))

(defparameter *base-frame* "/odom_combined")
(defparameter *left-eef-frame* "/l_wrist_roll_link")
(defparameter *right-eef-frame* "/r_wrist_roll_link")

(defparameter *eef-tool-transform-left* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.15 0 0)
                                                                      (cl-transforms:make-quaternion 0 0 0 1)))
(defparameter *eef-tool-transform-right* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.15 0 0)
                                                                       (cl-transforms:make-quaternion 0 0 0 1)))
(defparameter *left-eef-park* (cl-transforms-stamped:make-pose-stamped "/base_link" 0
                                                                       (cl-transforms:make-3d-vector 0.35 0.35 0.75)
                                                                       (cl-transforms:make-quaternion 0 0 0 1)))
(defparameter *right-eef-park* (cl-transforms-stamped:make-pose-stamped "/base_link" 0
                                                                       (cl-transforms:make-3d-vector 0.35 -0.35 0.75)
                                                                       (cl-transforms:make-quaternion 0 0 0 1)))
(defparameter *pizza-transform* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.7 0 0.6)
                                                              (cl-transforms:make-quaternion 0 0 0 1)))
(defparameter *plate-radius* 0.18)
(defparameter *from-pizza-to-plate* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0 0 -0.01)
                                                                  (cl-transforms:make-quaternion 0 0 0 1)))

(defparameter *cut-skeleton-wrapper* (create-cut-skeleton-wrapper (list 0.1 -0.15 0.01 0 0 0.01 0 0 0.01 0.07 0.17 0.01)
                                                                  (cl-transforms:make-transform (cl-transforms:make-3d-vector 0 0 0.12) (cl-transforms:make-quaternion 0 0 0 1))
                                                                  *pizza-transform*))

(defun init-place-object-demo-markers ()
  (pizza-ninja::place-plate-group-markers *pizza-transform* *from-pizza-to-plate* 0 1 2 "/odom_combined")
  (pizza-ninja::place-skeleton-markers *cut-skeleton-wrapper* "/odom_combined"))

(defun get-plate-grab-transform (arm-base-transform object-transform maneuver-arm)
  (let* ((robot-object-vector (cl-transforms-stamped:v- (cl-transforms:translation object-transform)
                                                        (cl-transforms:translation arm-base-transform)))
         (robot-object-direction (cl-transforms:normalize-vector robot-object-vector))
         (aux-angle (if (equal maneuver-arm :left)
                      (/ (- 0 pi) 18.0)
                      (/ pi 18.0)))
         (z-10-degree (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) aux-angle))
         (grab-translation (cl-transforms:v- (cl-transforms:translation object-transform)
                                             (cl-transforms:v* (cl-transforms:rotate z-10-degree robot-object-direction) *plate-radius*)))
         (grab-rotation z-10-degree)
         (grab-point (cl-transforms:make-transform grab-translation grab-rotation))
         (tool-point (cl-transforms:transform* grab-point (cl-transforms:transform-inv (if (equal maneuver-arm :left) 
                                                                                           *eef-tool-transform-left*
                                                                                           *eef-tool-transform-right*)))))
    tool-point))

(defun get-plate-grab (arm-base-transform object-transform maneuver-arm)
  (let* ((base-frame *base-frame*)
         (tool-point (get-plate-grab-transform arm-base-transform object-transform maneuver-arm))
         (grab-pose-stamped (cl-transforms-stamped:make-pose-stamped base-frame 0
                                                                     (cl-transforms:translation tool-point)
                                                                     (cl-transforms:rotation tool-point))))
    grab-pose-stamped))

(defun get-plate-release-angle-old (w z)
  (let* ((w (if (< z 0) w (- 0 w)))
         (half-angle (acos w))
         (angle (* half-angle 2))
         (angle (if (> 0 w)
                    (- angle (* 2 pi))
                    angle))
         (upper (/ pi 18))
         (lower (- 0 upper))
         (angle (if (< upper angle)
                  upper
                  angle))
         (angle (if (< angle lower)
                  lower
                  angle)))
    angle))

(defun put-in-angle-range (angle)
  (if (< angle (- 0 pi))
    (+ angle pi pi)
    (if (< pi angle)
      (- angle pi pi)
      angle)))

(defun get-plate-release-angle (object-transform suggested-transform)
  (let* ((object-angle (nth-value 1 (cl-transforms:quaternion->axis-angle (cl-transforms:rotation object-transform))))
         (suggested-angle (nth-value 1 (cl-transforms:quaternion->axis-angle (cl-transforms:rotation suggested-transform))))
         (object-angle (put-in-angle-range object-angle))
         (suggested-angle (put-in-angle-range suggested-angle))
         (angle (put-in-angle-range (- suggested-angle object-angle)))
         (upper (/ pi 9))
         (lower (- 0 upper))
         (angle (if (< upper angle)
                  upper
                  angle))
         (angle (if (< angle lower)
                  lower
                  angle))
         (dangle angle)
         (angle (put-in-angle-range (+ angle object-angle))))
    (values angle dangle)))

(defun get-plate-release (arm-base-transform object-transform suggested-transform maneuver-arm)
  (let* ((base-frame *base-frame*)
         (obj-tr-inv (cl-transforms:transform-inv object-transform))
         (angle (get-plate-release-angle object-transform suggested-transform))
         (dummy (format t "ANGLE ~a OBJ ~a SUGGESTED ~a~%" angle (cl-transforms:translation object-transform) (cl-transforms:translation suggested-transform)))
         (suggested-transform (cl-transforms:make-transform (cl-transforms:translation suggested-transform)
                                                            (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (- 0 angle))))
         (displacement-transform (cl-transforms:transform* suggested-transform obj-tr-inv))
         (tool-point (get-plate-grab-transform arm-base-transform object-transform maneuver-arm))
         (tool-point (cl-transforms:transform* displacement-transform tool-point))
         (release-pose-stamped (cl-transforms-stamped:make-pose-stamped base-frame 0
                                                                        (cl-transforms:translation tool-point)
                                                                        (cl-transforms:rotation tool-point))))
    release-pose-stamped))

(defun get-park (maneuver-arm)
  (if (equal maneuver-arm :left)
    *left-eef-park*
    *right-eef-park*))

(defun on-grab-plate (cut-skeleton-wrapper base-frame maneuver-arm tf-listener)
  (let* ((linked-frame (if (equal maneuver-arm :left)
                         *left-eef-frame*
                         *right-eef-frame*))
         (base-to-link-transform (cl-tf:lookup-transform tf-listener base-frame linked-frame))
         (where *pizza-transform*))
    (place-plate-group-markers where *from-pizza-to-plate* 0 1 2 base-frame :linked-frame linked-frame :base-to-link-transform base-to-link-transform)
    (place-skeleton-markers cut-skeleton-wrapper base-frame :linked-frame linked-frame :base-to-link-transform base-to-link-transform)))

(defun on-release-plate (cut-skeleton-wrapper base-frame pose-update)
  (setf *pizza-transform* (cl-transforms:transform* pose-update *pizza-transform*))
  (move-skeleton cut-skeleton-wrapper pose-update)
  (let* ((where *pizza-transform*))
    (place-plate-group-markers where *from-pizza-to-plate* 0 1 2 base-frame)
    (place-skeleton-markers cut-skeleton-wrapper base-frame)))

;;(defun ltfnp-follow-current-segment (cut-skeleton-wrapper &optional (tool-arm :right))
;;  (let* ((segment (get-current-segment cut-skeleton-wrapper))
;;         (seg-prestart (segment-prestart segment))
;;         (seg-prestart (cl-transforms-stamped:make-pose-stamped "/base_link" 0 (cl-transforms:translation seg-prestart) (cl-transforms:rotation seg-prestart)))
;;         (seg-start (segment-start segment))
;;         (seg-start (cl-transforms-stamped:make-pose-stamped "/base_link" 0 (cl-transforms:translation seg-start) (cl-transforms:rotation seg-start)))
;;         (seg-end (segment-end segment))
;;         (seg-end (cl-transforms-stamped:make-pose-stamped "/base_link" 0 (cl-transforms:translation seg-end) (cl-transforms:rotation seg-end)))
;;         (seg-postend (segment-postend segment))
;;         (seg-postend (cl-transforms-stamped:make-pose-stamped "/base_link" 0 (cl-transforms:translation seg-postend) (cl-transforms:rotation seg-postend))))
;;    (move-arm-pose tool-arm seg-prestart)
;;    (move-arm-pose tool-arm seg-start)
;;    (move-arm-pose tool-arm seg-end)
;;    (move-arm-pose tool-arm seg-postend)
;;    (pop-skeleton-segment cut-skeleton-wrapper)))

(defun place-object (object-transform arm-base-transform suggested-transform aux-arm get-grab get-release on-grab on-release &optional (accumulated-transform (cl-transforms:make-identity-transform)))
  (let* ((difference-transform (cl-transforms-stamped:transform* (cl-transforms-stamped:transform-inv object-transform) suggested-transform object-transform))
         (translation-distance (cl-transforms-stamped:v-norm (cl-transforms:v- (cl-transforms-stamped:translation suggested-transform) (cl-transforms-stamped:translation object-transform))))
         (x-dir (cl-transforms:make-3d-vector 1 0 0))
         (x-s (cl-transforms:rotate (cl-transforms:rotation suggested-transform) x-dir))
         (x-o (cl-transforms:rotate (cl-transforms:rotation object-transform) x-dir))
         (x-prod (cl-transforms:dot-product x-s x-o))
         (angle (abs (acos x-prod)))
         (grab-pose (funcall get-grab arm-base-transform object-transform suggested-transform aux-arm))
         (release-pose (funcall get-release arm-base-transform object-transform suggested-transform aux-arm))
         (park-pose (get-park :left))
         (pose-update (cl-transforms-stamped:transform* (cl-transforms-stamped:pose->transform release-pose) (cl-transforms-stamped:transform-inv (cl-transforms-stamped:pose->transform grab-pose))))
         (new-object-transform (cl-transforms-stamped:transform* pose-update object-transform))
         (accumulated-transform (cl-transforms-stamped:transform* pose-update accumulated-transform)))
    (format t "TRANFORM DISTANCE ~a ~a~%" translation-distance angle)
    (if (or (< 0.01 translation-distance) (< 0.1 angle))
      (progn
;;(format t "Grabbing at pose ~a ~%" grab-pose)
;;(format t "Release at pose ~a ~%" release-pose)
;;(format t "Update pose ~a ~%" pose-update)
;;(format t "Accumulated transform ~a ~%" accumulated-transform)
        (move-arm-pose aux-arm grab-pose)
        (apply on-grab nil)
        (move-arm-pose aux-arm release-pose)
        (apply on-release (list pose-update))
        (move-arm-pose aux-arm park-pose)
        (place-object new-object-transform arm-base-transform suggested-transform aux-arm get-grab get-release on-grab on-release accumulated-transform)
        )
      accumulated-transform)))


(defun test-place-pizza ()
  (let* ((pr2-r-arm-capmap (make-instance 'pr2-reachability-costmap:reachability-map :group-name "right_arm" :ik-base-name "torso_lift_link" :ik-link-name "r_wrist_roll_link" :filename "/home/blandc/catkin_ws/src/cram_pr2/pr2_reachability_costmap/resource/pr2-reachability-map-right-5cm.map"))
         (pose-suggestions (suggest-placement-transform *cut-skeleton-wrapper* :right (cl-tf:lookup-transform cram-moveit::*transformer* "/odom_combined" "/torso_lift_link") :reachability-map pr2-r-arm-capmap))
         (suggested-transform (car (car pose-suggestions)))
         (suggested-transform-viz (cl-transforms:make-transform (cl-transforms:v+ (cl-transforms:translation suggested-transform)
                                                                                  (cl-transforms:make-3d-vector 0 0 0.05))
                                                                (cl-transforms:rotation suggested-transform)))
         )
    (move-arm-pose :left *left-eef-park*)
    (move-arm-pose :right *right-eef-park*)
    (place-plate-group-markers suggested-transform-viz *from-pizza-to-plate* 10 11 12 "/odom_combined" :alpha 0.5)
    (roslisp:wait-duration 1)
    (place-reachmap-markers pose-suggestions "/odom_combined" *pizza-transform*)
    (place-object *pizza-transform*
                  (cl-tf:lookup-transform cram-moveit::*transformer* "/odom_combined" "/torso_lift_link")
                  suggested-transform
                  :left
                  (lambda (arm-base-transform object-transform suggested-transform maneuver-arm)
                    (declare (ignore suggested-transform))
                    (get-plate-grab arm-base-transform object-transform maneuver-arm))
                  (lambda (arm-base-transform object-transform suggested-transform maneuver-arm)
                    (get-plate-release arm-base-transform object-transform suggested-transform maneuver-arm))
                  (lambda ()
                    (on-grab-plate *cut-skeleton-wrapper* "/odom_combined" :left cram-moveit::*transformer*))
                  (lambda (pose-update)
                    (on-release-plate *cut-skeleton-wrapper* "/odom_combined" pose-update)))))


;(cpl-impl:def-top-level-cram-function setup-pr2 ()
;  (let* ((torso-action (actionlib:make-action-client "/torso_controller/position_joint_action"
;                                                     "pr2_controllers_msgs/SingleJointPositionAction")))
;    (cpl-impl:par
;      (progn
;        (actionlib:wait-for-server torso-action)
;        (actionlib:send-goal-and-wait torso-action
;                                      (actionlib:make-action-goal torso-action position 0.3)
;                                      :result-timeout 30.0
;                                      :exec-timeout 30.0))
;      (progn
;        (ltfnp::move-arms-up)
;        (actionlib-lisp:send-goal-and-wait pr2-nav-pm::*navp-client*
;                                           (pr2-nav-pm::make-action-goal (cl-transforms-stamped:make-pose-stamped "odom_combined" 0
;                                                                                                                  (cl-transforms:make-3d-vector -0.2 1.5 0)
;                                                                                                                  (cl-transforms:make-quaternion 0 0 1 0)))
;                                           100 100)))))

