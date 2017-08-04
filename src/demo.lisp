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

(defparameter *left-arm-up*
              (tf:make-pose-stamped
                "torso_lift_link" 0.0
                (tf:make-3d-vector 0.1 0.45 0.3)
                (tf:euler->quaternion :ay (/ pi -2))))

(defparameter *right-arm-up*
              (tf:make-pose-stamped
                "torso_lift_link" 0.0
                (tf:make-3d-vector 0.1 -0.45 0.3)
                (tf:euler->quaternion :ay (/ pi -2))))

(defparameter *pose-cutter-grab*
              (cl-transforms-stamped:make-pose-stamped "torso_lift_link" 0.0
                                                       (cl-transforms:make-3d-vector 0.6 0.25 0.03)
                                                       (cl-transforms:euler->quaternion :ay (/ pi 2))))

(defparameter *pose-cutter-pregrab*
              (cl-transforms-stamped:make-pose-stamped "torso_lift_link" 0.0
                                                       (cl-transforms:make-3d-vector 0.6 0.25 0.15)
                                                       (cl-transforms:euler->quaternion :ay (/ pi 2))))

(defparameter *pose-left-handover*
              (cl-transforms-stamped:make-pose-stamped "torso_lift_link" 0.0
                                                       (cl-transforms:make-3d-vector 0.55 0.2 0.20)
                                                       (cl-transforms:euler->quaternion :az (/ pi -2))))

(defparameter *pose-right-handover*
              (cl-transforms-stamped:make-pose-stamped "torso_lift_link" 0.0
                                                       (cl-transforms:make-3d-vector 0.55 -0.2 0.10)
                                                       (cl-transforms:euler->quaternion :az (/ pi 2))))

(defparameter *above-grab-transform-left* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.0 0 0.22)
                                                                      (cl-transforms:euler->quaternion :ay (/ pi 2))))
(defparameter *above-grab-transform-right* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.0 0 0.22)
                                                                       (cl-transforms:euler->quaternion :ay (/ pi 2))))
;;;;;; !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
(defparameter *side-grab-transform-left* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.0 -0.2 0.0)
                                                                      (cl-transforms:euler->quaternion :az (/ pi 2))))
(defparameter *side-grab-transform-right* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.0 0.2 0.0)
                                                                       (cl-transforms:euler->quaternion :az (/ pi -2))))
(defparameter *left-eef-park* (cl-transforms-stamped:make-pose-stamped "torso_lift_link" 0
                                                                       (cl-transforms:make-3d-vector 0.35 0.35 0.15)
                                                                       (cl-transforms:make-quaternion 0 0 0 1)))
(defparameter *right-eef-park* (cl-transforms-stamped:make-pose-stamped "torso_lift_link" 0
                                                                       (cl-transforms:make-3d-vector 0.35 -0.35 0.15)
                                                                       (cl-transforms:make-quaternion 0 0 0 1)))
(defparameter *plate-radius* 0.2)
(defparameter *bread-length* 0.3)
(defparameter *bread-width* 0.07)

(defparameter *cut-skeleton-wrapper* (create-cut-skeleton-wrapper (list 0.1 -0.15 0.01 0 0 0.01 0 0 0.01 0.07 0.17 0.01)
                                                                  (cl-transforms:make-identity-transform)
                                                                  (cl-transforms:make-identity-transform)))


(defparameter *initial-pose-pizza* nil)
(defparameter *initial-pose-bread* nil)
(defparameter *initial-pose-knife* nil)
(defparameter *initial-pose-pizza-cutter* nil)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun get-object-semmap-name (object-name)
  (cond
    ((equal object-name "pizza_plate") "http://knowrob.org/kb/IAI-kitchen.owl#Pizza_sfd49bhxbf")
    ((equal object-name "pizza_cutter") "http://knowrob.org/kb/IAI-kitchen.owl#PizzaCutter_sfd49bhxbf")
    ((equal object-name "bread") "http://knowrob.org/kb/IAI-kitchen.owl#Bread_sfd49bhxbf")
    ((equal object-name "knife") "http://knowrob.org/kb/IAI-kitchen.owl#Knife_sfd49bhxbf")
    (T "")))

;; Basic move functions

(defun move-arm-poses (arm poses)
  (cpl-impl:with-failure-handling
    ((cram-plan-failures:manipulation-pose-unreachable (f)
       (declare (ignore f))
       ;; So far, when encountered this meant the simulation was not recoverable
       ;; This will (assuming we're running from a sim_inst_mngr.py script) cause the entire simulation to eventually reset
       (setf (cpl:value prac2cram:plan-error) T)
       ;; Put a retry here, but we're really just waiting for SIGTERM from sim_inst_mngr.py
       (cpl-impl:retry)))
    (mot-man:execute-arm-action (mot-man:make-goal-specification
                                  :moveit-goal-specification
                                  :arm-pose-goals (list (list arm (mot-man:eef-link-name arm) poses))))))

(defun move-arms-up ()
  (cpl-impl:with-failure-handling
    ((cram-plan-failures:manipulation-pose-unreachable (f)
       (declare (ignore f))
       ;; So far, when encountered this meant the simulation was not recoverable
       ;; This will (assuming we're running from a sim_inst_mngr.py script) cause the entire simulation to eventually reset
       (setf (cpl:value prac2cram:plan-error) T)
       ;; Put a retry here, but we're really just waiting for SIGTERM from sim_inst_mngr.py
       (cpl-impl:retry)))
    (mot-man:execute-arm-action (cram-moveit-manager:make-goal-specification
                                  :moveit-goal-specification
                                  :keys '((:raise-elbow (:left :right)))
                                  :arm-pose-goals (list `(:left ,*left-arm-up*)
                                                        `(:right ,*right-arm-up*))))))

;; Load capmap

(defun get-arm-capmap (arm)
  (cond
    ((equal arm :right)
      (make-instance 'pr2-reachability-costmap:reachability-map
                     :group-name "right_arm"
                     :ik-base-name "torso_lift_link"
                     :ik-link-name "r_wrist_roll_link"
                     :filename (format nil "~a/resources/pr2-reachability-map-right-5cm.map"
                                           (namestring (ros-load:ros-package-path "pizza_demo")))))
    ((equal arm :left)
      (make-instance 'pr2-reachability-costmap:reachability-map
                     :group-name "left_arm"
                     :ik-base-name "torso_lift_link"
                     :ik-link-name "l_wrist_roll_link"
                     :filename (format nil "~a/resources/pr2-reachability-map-left-5cm.map"
                                           (namestring (ros-load:ros-package-path "pizza_demo")))))))

;; Arm selection

(defun get-tool-grabbing-arm (object-name tool-name tf-transformer)
  (declare (ignore tool-name) (ignore tf-transformer))
  (cond
    ((equal object-name "pizza_plate")
      :left)
    ((equal object-name "bread")
      :right)))

(defun get-maneuver-arm (object-name tool-name tf-transformer)
  (declare (ignore tool-name) (ignore tf-transformer))
  (cond
    ((equal object-name "pizza_plate")
      :right)
    ((equal object-name "bread")
      :right)))

(defun get-aux-arm (object-name tool-name tf-transformer)
  (declare (ignore tool-name) (ignore tf-transformer))
  (cond
    ((equal object-name "pizza_plate")
      :left)
    ((equal object-name "bread")
      :left)))

;; Pose selection

;;    Base

(defun get-desired-base-pose (object-name tf-transformer)
;;;;;;;;;;;;;;;;;; !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Adjust based on actual pose of the object
;;;;;;;;;;;;;;;;;; Also then reposition the object at the end, close to where it used to be.
  (declare (ignore tf-transformer))
  (cond
    ((equal object-name "pizza_plate")
      (cl-transforms-stamped:make-pose-stamped "map" 0
                                               (cl-transforms:make-3d-vector -0.25 1.85 0)
                                               (cl-transforms:make-quaternion 0 0 1 0)))
    ((equal object-name "bread")
      (cl-transforms-stamped:make-pose-stamped "map" 0
                                               (cl-transforms:make-3d-vector -0.25 1.25 0)
                                               (cl-transforms:make-quaternion 0 0 1 0)))))

;;    Tool relative to skeleton

(defun get-skeleton-to-tool (tool-name)
  (cond
    ((equal tool-name "pizza_cutter")
      (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.195 0 0.15) (cl-transforms:make-quaternion 0 0 0 1)))
    ((equal tool-name "knife")
      (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.22 0 0.195) (cl-transforms:euler->quaternion :ay (/ pi 2))))))

;;    Tool frame placement in arm

(defun get-grab-transform (arm grab-dir)
  (cond
    ((equal grab-dir :above)
      (if (equal arm :left) 
        *above-grab-transform-left*
        *above-grab-transform-right*))
    ((equal grab-dir :side)
      (if (equal arm :left) 
        *side-grab-transform-left*
        *side-grab-transform-right*))))

;;    Tool placement in world

(defun get-near-object-distance (object-name)
  (cond
    ((equal object-name "pizza_plate")
      (+ *plate-radius* 0.05))
    ((equal object-name "bread")
      (+ (* *bread-length* 0.5) 0.05))))

(defun get-tool-place-locs (arm object-name tool-name object-loc)
  (let* ((object-near (get-near-object-distance object-name))
         (tool-pos (cl-transforms:v+ (cl-transforms:translation object-loc)
                                     (if (equal arm :left)
                                       (cl-transforms:make-3d-vector 0.06 (+ 0 object-near) 0)
                                       (cl-transforms:make-3d-vector 0.06 (- 0 object-near) 0))))
         (tool-loc (cl-transforms-stamped:make-transform-stamped
                     (cl-transforms-stamped:frame-id object-loc) tool-name 0
                     tool-pos
                     (if (equal tool-name "pizza_cutter")
                       (cl-transforms:euler->quaternion :ay (/ pi -2))
                       (cl-transforms:euler->quaternion)))))
    tool-loc))

;;    Arm poses

(defun get-park-pose (arm)
  (cond
    ((equal :left arm)
      *left-eef-park*)
    ((equal :right arm)
      *right-eef-park*)))

(defun get-prepose (pose world-vector-id from-direction &key (disp-step 0.05))
  (let* ((frame-id (cl-transforms-stamped:frame-id pose))
         (stamp (cl-transforms-stamped:stamp pose))
         (rotation (cl-transforms-stamped:orientation pose))
         (translation (cl-transforms-stamped:origin pose))
         (disp-step (if (equal from-direction :above)
                      disp-step
                      (- 0 disp-step)))
         (disp-vector (cond
                        ((equal world-vector-id :x)
                          (cl-transforms:make-3d-vector disp-step 0 0))
                        ((equal world-vector-id :y)
                          (cl-transforms:make-3d-vector 0 disp-step 0))
                        ((equal world-vector-id :z)
                          (cl-transforms:make-3d-vector 0 0 disp-step))))
         (translation (cl-transforms:v+ translation
                                        disp-vector)))
    (cl-transforms-stamped:make-pose-stamped frame-id stamp
                                             translation rotation)))

(defun get-tool-grab-point (tool-name arm)
  (cond
    ((equal tool-name "pizza_cutter")
      (if (equal arm :left)
        (cl-transforms:make-3d-vector 0 0 0.22)
        (cl-transforms:make-3d-vector 0 0 0.12)))
    ((equal tool-name "knife")
      (if (equal arm :left)
        (cl-transforms:make-3d-vector 0.05 0 0)
        (cl-transforms:make-3d-vector 0 0 0)))))

;;;;;;; !!!!!!!!!!!!!!!!!!!!!
(defun get-arm-grab-locs-old (arm tool-name tool-loc arm-grab-type)
  (let* ((tool-point (cl-transforms:make-transform (get-tool-grab-point tool-name arm)
                                                   (cl-transforms:euler->quaternion)))
         (tool-point (cl-transforms:transform* tool-loc tool-point))
         (tool-point (cl-transforms:make-transform (cl-transforms:translation tool-point)
                                                   (cl-transforms:euler->quaternion)))
         (approach (if (or (equal arm-grab-type :pickup) (equal tool-name "knife"))
                     :above
                     :side))
         (grab-transform (get-grab-transform arm approach))
         (tool-point (cl-transforms:transform* tool-point grab-transform))
         (grab-translation (cl-transforms:translation tool-point))
         (grab-rotation (cl-transforms:rotation tool-point))
         (grab (cl-transforms-stamped:make-pose-stamped "map" 0 grab-translation grab-rotation))
         (approach-v (cond
                       ((equal tool-name "pizza_cutter")
                         (if (equal arm-grab-type :pickup)
                           :z
                           :y))
                       ((equal tool-name "knife")
                         :z)))
         (approach-dir (cond
                         ((equal tool-name "pizza_cutter")
                           (if (equal arm-grab-type :pickup)
                             :above
                             (if (equal arm :left)
                               :under
                               :above)))
                         ((equal tool-name "knife")
                           :above)))
         (pregrab (get-prepose grab approach-v approach-dir)))
    (list pregrab grab)))

(defun get-arm-grab-locs (arm tool-name tool-loc arm-grab-type)
  (let* ((pregrab (cond
                    ((equal arm :left)
                      (cond
                        ((equal tool-name "pizza_cutter")
                          (cond
                            ((equal arm-grab-type :pickup)
                              (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.275 0 0.22) (cl-transforms:euler->quaternion :az pi)))
                            ((equal arm-grab-type :use)
                              (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.275 0 0.22) (cl-transforms:euler->quaternion)))))
                        ((equal tool-name "knife")
                          (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.22 0 0.29) (cl-transforms:euler->quaternion :ay (/ pi 2))))))
                    ((equal arm :right)
                      (cond
                        ((equal tool-name "pizza_cutter")
                          (cond
                            ((equal arm-grab-type :pickup)
                              (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.275 0 0.12) (cl-transforms:euler->quaternion :az pi)))
                            ((equal arm-grab-type :use)
                              (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.275 0 0.12) (cl-transforms:euler->quaternion)))))
                        ((equal tool-name "knife")
                          (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.12 0 0.29) (cl-transforms:euler->quaternion :ay (/ pi 2))))))))
         (grab (cond
                 ((equal arm :left)
                   (cond
                     ((equal tool-name "pizza_cutter")
                       (cond
                         ((equal arm-grab-type :pickup)
                           (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.2 0 0.22) (cl-transforms:euler->quaternion :az pi)))
                         ((equal arm-grab-type :use)
                           (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.2 0 0.22) (cl-transforms:euler->quaternion)))))
                     ((equal tool-name "knife")
                       (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.22 0 0.22) (cl-transforms:euler->quaternion :ay (/ pi 2))))))
                 ((equal arm :right)
                   (cond
                     ((equal tool-name "pizza_cutter")
                       (cond
                         ((equal arm-grab-type :pickup)
                           (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.2 0 0.12) (cl-transforms:euler->quaternion :az pi)))
                         ((equal arm-grab-type :use)
                           (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.2 0 0.12) (cl-transforms:euler->quaternion)))))
                     ((equal tool-name "knife")
                       (cl-transforms:make-transform (cl-transforms:make-3d-vector -0.12 0 0.22) (cl-transforms:euler->quaternion :ay (/ pi 2))))))))
         (pregrab (cl-transforms:transform* tool-loc pregrab))
         (grab (cl-transforms:transform* tool-loc grab))
         (pregrab (cl-transforms-stamped:make-pose-stamped (cl-transforms-stamped:frame-id tool-loc) 0
                                                           (cl-transforms-stamped:translation pregrab)
                                                           (cl-transforms-stamped:rotation pregrab)))
         (grab (cl-transforms-stamped:make-pose-stamped (cl-transforms-stamped:frame-id tool-loc) 0
                                                        (cl-transforms-stamped:translation grab)
                                                        (cl-transforms-stamped:rotation grab))))
    (list pregrab
          grab)))

(defun get-handover-src-pose (arm)
  (cond
    ((equal arm :left)
      *pose-left-handover*)
    ((equal arm :right)
      *pose-right-handover*)))

;; Marker handling

(defun get-mesh-resource (object-name)
  (cond
    ((equal object-name "pizza_plate")
      "package://pizza_demo/models/pizza_plate/meshes/pizza_plate_visual.stl")
    ((equal object-name "bread")
      "package://pizza_demo/models/bread/meshes/bread.stl")))

(defun reset-skeleton-markers ()
  (roslisp:publish (ensure-mrk-publisher) (roslisp:make-message "visualization_msgs/Marker"
                                                                :header (roslisp:make-message "std_msgs/Header" :frame_id "map" :stamp 0)
                                                                :id 0
                                                                :action 3
                                                                :ns "cut-skeleton")))

(defun update-markers (cut-skeleton-wrapper object-name tool-name slices-marker tf-transformer)
  (declare (ignore tool-name) (ignore slices-marker))
  (place-skeleton-markers cut-skeleton-wrapper
                          "map"
                          :linked-frame object-name
                          :base-to-link-transform (cl-tf:lookup-transform tf-transformer "map" object-name)))

(defun place-object-group-markers (mrk-namespace object-name slices-marker &key location (alpha 1.0))
  (let* ((base-frame (if location
                       "map"
                       object-name))
         (frame-locked (if location 0 1))
         (obj-pose-msg (if location
                         (roslisp:make-message "geometry_msgs/Pose"
                                               :position (roslisp:make-message "geometry_msgs/Point" :x (cl-transforms:x (cl-transforms:translation location))
                                                                                                     :y (cl-transforms:y (cl-transforms:translation location))
                                                                                                     :z (cl-transforms:z (cl-transforms:translation location)))
                                               :orientation (roslisp:make-message "geometry_msgs/Quaternion" :x (cl-transforms:x (cl-transforms:rotation location))
                                                                                                             :y (cl-transforms:y (cl-transforms:rotation location))
                                                                                                             :z (cl-transforms:z (cl-transforms:rotation location))
                                                                                                             :w (cl-transforms:w (cl-transforms:rotation location))))
                         (roslisp:make-message "geometry_msgs/Pose"
                                               :position (roslisp:make-message "geometry_msgs/Point" :x 0 :y 0 :z 0)
                                               :orientation (roslisp:make-message "geometry_msgs/Quaternion" :x 0 :y 0 :z 0 :w 1))))
         (object-msg (make-mrk-msg base-frame frame-locked 0 0 obj-pose-msg *object-color* (get-mesh-resource object-name) :alpha alpha :namespace mrk-namespace))
         (slice-rotation (when slices-marker
                           (second slices-marker)))
         (slice-rotation (when slices-marker
                           (cl-transforms:make-transform (cl-transforms:make-3d-vector 0 0 0) (cl-transforms:euler->quaternion :az slice-rotation))))
         (slice-location (when slices-marker
                           (if location
                             location
                             (cl-transforms:make-identity-transform))))
         (slice-location (when slices-marker
                           (cl-transforms:transform* slice-location slice-rotation)))
         (slice-pose-msg (when slices-marker
                           (roslisp:make-message "geometry_msgs/Pose"
                                                 :position (roslisp:make-message "geometry_msgs/Point" :x (cl-transforms:x (cl-transforms:translation slice-location))
                                                                                                       :y (cl-transforms:x (cl-transforms:translation slice-location))
                                                                                                       :z (cl-transforms:x (cl-transforms:translation slice-location)))
                                                 :orientation (roslisp:make-message "geometry_msgs/Quaternion" :x (cl-transforms:x (cl-transforms:rotation slice-location))
                                                                                                               :y (cl-transforms:y (cl-transforms:rotation slice-location))
                                                                                                               :z (cl-transforms:z (cl-transforms:rotation slice-location))
                                                                                                               :w (cl-transforms:w (cl-transforms:rotation slice-location))))))
         (slice-msg (when slices-marker
                      (make-mrk-msg base-frame frame-locked 1 0 slice-pose-msg *slice-color* (first slices-marker) :alpha alpha :namespace mrk-namespace))))
    (roslisp:publish (ensure-mrk-publisher) object-msg)
    (when slice-msg
      (roslisp:publish (ensure-mrk-publisher) slice-msg))))

;; Validate pose suggestions

(defun get-valid-pose-suggestion (pose-suggestions)
  (car (car pose-suggestions)))

;; Handle object grab/release events

(defun attach-model (robot-model-name robot-link-name object-model-name object-link-name)
  (roslisp:call-service "/gazebo/attach" 'attache_msgs-srv:Attachment
                        :model1 robot-model-name
                        :link1 robot-link-name
                        :model2 object-model-name
                        :link2 object-link-name))

(defun detach-model (robot-model-name robot-link-name object-model-name object-link-name)
  (roslisp:call-service "/gazebo/detach" 'attache_msgs-srv:Attachment
                        :model1 robot-model-name
                        :link1 robot-link-name
                        :model2 object-model-name
                        :link2 object-link-name))

(defun on-grab-object (object-name arm)
  (detach-model "IAI_kitchen" "room_link" object-name object-name)
  (attach-model "pr2" (mot-man:eef-link-name arm) object-name object-name))

(defun on-release-object (object-name arm)
  (detach-model "pr2" (mot-man:eef-link-name arm) object-name object-name)
  (attach-model "IAI_kitchen" "room_link" object-name object-name))

;; Compute grasp/release poses for repositioning maneuvers

(defun get-plate-grab-transform (arm-base-transform object-transform maneuver-arm)
  (let* ((robot-object-vector (cl-transforms-stamped:v- (cl-transforms:translation object-transform)
                                                        (cl-transforms:translation arm-base-transform)))
         (robot-object-vector (cl-transforms:make-3d-vector (cl-transforms:x robot-object-vector) (cl-transforms:y robot-object-vector) 0.0))
         (robot-object-direction (cl-transforms:normalize-vector robot-object-vector))
         (aux-angle (if (equal maneuver-arm :left)
                      (/ (- 0 pi) 18.0)
                      (/ pi 18.0)))
         (z-10-degree (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) aux-angle))
         (grab-translation (cl-transforms:v- (cl-transforms:translation object-transform)
                                             (cl-transforms:v* (cl-transforms:rotate z-10-degree robot-object-direction) *plate-radius*)))
         (grab-rotation z-10-degree)
         (grab-point (cl-transforms:make-transform grab-translation grab-rotation))
         (tool-point (cl-transforms:transform* grab-point (get-grab-transform maneuver-arm :above))))
    tool-point))

(defun get-plate-grab (arm-base-transform object-transform maneuver-arm)
  (let* ((base-frame "torso_lift_link")
         (object-transform (cl-transforms:transform* (cl-transforms:transform-inv arm-base-transform) object-transform))
         (arm-base-transform (cl-transforms:make-identity-transform))
         (tool-point (get-plate-grab-transform arm-base-transform object-transform maneuver-arm))
         (grab-pose-stamped (cl-transforms-stamped:make-pose-stamped base-frame 0
                                                                     (cl-transforms:translation tool-point)
                                                                     (cl-transforms:rotation tool-point))))
    grab-pose-stamped))

(defun get-bread-grab (arm-base-transform object-transform maneuver-arm)
  (let* ((base-frame "torso_lift_link")
         (object-transform (cl-transforms:transform* (cl-transforms:transform-inv arm-base-transform) object-transform))
         (grab-point (cl-transforms:make-transform (cl-transforms:v+
                                                     (cl-transforms:translation object-transform)
                                                     (cl-transforms:rotate (cl-transforms:rotation object-transform)
                                                                           (cl-transforms:make-3d-vector (- 0 (* *bread-length* 0.5)) 0 0)))
                                                   (cl-transforms:rotation object-transform)))
         (tool-point (cl-transforms:transform* grab-point
                                               (get-grab-transform maneuver-arm :above)))
         (grab-pose-stamped (cl-transforms-stamped:make-pose-stamped base-frame 0
                                                                     (cl-transforms:translation tool-point)
                                                                     (cl-transforms:rotation tool-point))))
    grab-pose-stamped))

(defun put-in-angle-range (angle)
  (if (< angle (- 0 pi))
    (put-in-angle-range (+ angle pi pi))
    (if (< pi angle)
      (put-in-angle-range (- angle pi pi))
      angle)))

(defun get-object-release-angle (object-transform suggested-transform)
  (let* ((object-angle (nth-value 1 (cl-transforms:quaternion->axis-angle (cl-transforms:rotation object-transform))))
         (object-axis (cl-transforms:quaternion->axis-angle (cl-transforms:rotation object-transform)))
         (suggested-angle (nth-value 1 (cl-transforms:quaternion->axis-angle (cl-transforms:rotation suggested-transform))))
         (suggested-axis (cl-transforms:quaternion->axis-angle (cl-transforms:rotation suggested-transform)))
         (suggested-angle (if (< (cl-transforms:z suggested-axis) 0)
                            (- 0 suggested-angle)
                            suggested-angle))
         (object-angle (if (< (cl-transforms:z object-axis) 0)
                         (- 0 object-angle)
                         object-angle))
         (object-angle (put-in-angle-range object-angle))
         (suggested-angle (put-in-angle-range suggested-angle))
         (angle (put-in-angle-range (- suggested-angle object-angle)))
         (upper (/ pi 5))
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

(defun get-object-reposition-grab (object-name object-loc robot-loc suggested-transform maneuver-arm aux-arm)
  (declare (ignore suggested-transform) (ignore maneuver-arm))
  (cond
    ((equal object-name "pizza_plate")
      (get-plate-grab robot-loc object-loc aux-arm))
    ((equal object-name "bread")
      (get-bread-grab robot-loc object-loc aux-arm))))

(defun get-object-reposition-release (object-name object-loc robot-loc suggested-transform maneuver-arm aux-arm)
  (let* ((base-frame "torso_lift_link")
         (object-loc (cl-transforms:transform* (cl-transforms:transform-inv robot-loc) object-loc))
         (suggested-transform (cl-transforms:transform* (cl-transforms:transform-inv robot-loc) suggested-transform))
         (robot-loc (cl-transforms:make-identity-transform))
         (obj-tr-inv (cl-transforms:transform-inv object-loc))
;;;; !!!! Currently this assumes that objects' vertical axis is aligned to world-z. Should change in the future
;;;; to handle repositioning of other object orientations, and to other repositionings, not just rotation around z axis.
         (angle (get-object-release-angle object-loc suggested-transform))
         (suggested-transform (cl-transforms:make-transform (cl-transforms:translation suggested-transform)
                                                            (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) angle)))
         (displacement-transform (cl-transforms:transform* suggested-transform obj-tr-inv))
         (tool-point (get-object-reposition-grab object-name object-loc robot-loc suggested-transform maneuver-arm aux-arm))
         (tool-point (cl-transforms:make-transform (cl-transforms:origin tool-point)
                                                   (cl-transforms:orientation tool-point)))
         (tool-point (cl-transforms:transform* displacement-transform tool-point))
         (release-pose-stamped (cl-transforms-stamped:make-pose-stamped base-frame 0
                                                                        (cl-transforms:translation tool-point)
                                                                        (cl-transforms:rotation tool-point))))
    release-pose-stamped))

;; Repositioning maneuvers

(defun close-enough (transform-a transform-b &key (translation-threshold 0.05) (angle-threshold 0.2))
  (let* ((translation-distance (cl-transforms-stamped:v-norm 
                                 (cl-transforms:v- (cl-transforms-stamped:translation transform-a)
                                                   (cl-transforms-stamped:translation transform-b))))
         (x-dir (cl-transforms:make-3d-vector 1 0 0))
         (x-s (cl-transforms:rotate (cl-transforms:rotation transform-a) x-dir))
         (x-o (cl-transforms:rotate (cl-transforms:rotation transform-b) x-dir))
         (x-prod (cl-transforms:dot-product x-s x-o))
         (angle (abs (acos x-prod))))
    (and (> translation-threshold translation-distance) (> angle-threshold angle))))

(defun reposition-object (object-name object-loc arm-capmap-loc suggested-transform maneuver-arm aux-arm tf-transformer &optional (init-sup-man-count 0))
  (let* ((robot-loc (cl-tf:lookup-transform tf-transformer "map" "torso_lift_link"))
         (grab-pose (get-object-reposition-grab object-name object-loc robot-loc suggested-transform maneuver-arm aux-arm))
         (release-pose (get-object-reposition-release object-name object-loc robot-loc suggested-transform maneuver-arm aux-arm))
         (pregrab-pose (get-prepose grab-pose :z :above))
         (prerelease-pose (get-prepose release-pose :z :above)))
    (unless (close-enough suggested-transform object-loc)
      (move-arm-poses aux-arm (list pregrab-pose grab-pose))
      (on-grab-object object-name aux-arm)
      (move-arm-poses aux-arm (list pregrab-pose prerelease-pose))
      (move-arm-poses aux-arm (list release-pose))
      (on-release-object object-name aux-arm)
      (move-arm-poses aux-arm prerelease-pose)
      (setf init-sup-man-count
            (reposition-object object-name (cl-tf:lookup-transform tf-transformer "map" object-name)
                               arm-capmap-loc suggested-transform maneuver-arm aux-arm tf-transformer (+ init-sup-man-count 1))))
    init-sup-man-count))


;; Bring PR2 to a nice location for the task

(defun setup-pr2 (desired-base-pose)
  (let* ((torso-action (actionlib:make-action-client "/torso_controller/position_joint_action"
                                                     "pr2_controllers_msgs/SingleJointPositionAction")))
    (cpl-impl:par
      (cpl-impl:with-failure-handling
        ((sb-sys:deadline-timeout (f)
           (declare (ignore f))
           (cpl-impl:retry)))
        (actionlib:wait-for-server torso-action)
        (actionlib:send-goal-and-wait torso-action
                                      (actionlib:make-action-goal torso-action position 0.3)
                                      :result-timeout 30.0
                                      :exec-timeout 30.0))
      (progn
        (move-arms-up)
        (actionlib-lisp:send-goal-and-wait pr2-nav-pm::*navp-client*
                                           (pr2-nav-pm::make-action-goal desired-base-pose)
                                           100 100)))))

;; Bigger plans

(defun handover-tool (tool-name object-name first-arm second-arm tf-transformer first-arm-has-tool put-down)
  (let* ((tool-loc (cl-tf:lookup-transform tf-transformer "torso_lift_link" tool-name))
         (first-arm-grab-locs (get-arm-grab-locs first-arm tool-name tool-loc :pickup))
         (first-arm-pregrab (first first-arm-grab-locs))
         (first-arm-grab (second first-arm-grab-locs))
         (robot-name "pr2")
         (first-arm-eef-link (mot-man:eef-link-name first-arm))
         (second-arm-eef-link (mot-man:eef-link-name second-arm)))
;; Grab the tool with the first arm, unless it's already grabbed
    (unless first-arm-has-tool
      (move-arm-poses first-arm (list first-arm-pregrab first-arm-grab))
      (detach-model "IAI_kitchen" "room_link" tool-name tool-name)
      (attach-model robot-name first-arm-eef-link tool-name tool-name)
      (move-arm-poses first-arm first-arm-pregrab))
;; Depending on whether we need to change which arm has the tool, go to a handover or park pose
    (if (equal first-arm second-arm)
      (move-arm-poses first-arm (list (get-park-pose first-arm)))
      (move-arm-poses first-arm (list (get-handover-src-pose first-arm))))
;; If needed, do the handover here
    (when (not (equal first-arm second-arm))
      (let* ((tool-loc (cl-tf:lookup-transform tf-transformer "torso_lift_link" tool-name))
             (arm-grab-type (if put-down
                              :pickup ;; not a typo :P
                              :use))
             (second-arm-grab-locs (get-arm-grab-locs second-arm tool-name tool-loc arm-grab-type))
             (second-arm-pregrab (first second-arm-grab-locs))
             (second-arm-grab (second second-arm-grab-locs)))
;; Bring the second arm near the tool
        (move-arm-poses second-arm (list second-arm-pregrab second-arm-grab))
;; Switch which arm carries the tool
        (attach-model robot-name second-arm-eef-link tool-name tool-name)
        (detach-model robot-name first-arm-eef-link tool-name tool-name)
;; Park the first arm, in case of the handover
        (move-arm-poses first-arm (list (get-park-pose first-arm)))))
;; Depending on whether we need to do a put-down, either do the put-down, or just park the arm
    (if put-down
      (let* ((object-loc (cl-tf:lookup-transform tf-transformer "torso_lift_link" object-name))
             (tool-loc (get-tool-place-locs second-arm object-name tool-name object-loc))
             (place-locs (get-arm-grab-locs second-arm tool-name tool-loc :pickup))
             (preplace-pose (first place-locs))
             (place-pose (second place-locs)))
        (move-arm-poses second-arm (list preplace-pose place-pose))
        (attach-model "IAI_kitchen" "room_link" tool-name tool-name)
        (detach-model robot-name second-arm-eef-link tool-name tool-name)
        (move-arm-poses second-arm (list (get-park-pose second-arm))))
      (move-arm-poses second-arm (list (get-park-pose second-arm))))))

(defun get-seg-waypoints (start end step)
  (let* ((start-pos (cl-transforms:origin start))
         (end-pos (cl-transforms:origin end))
         (seg (cl-transforms:v- end-pos start-pos))
         (dist (cl-transforms:v-norm seg))
         (step-adj (/ step dist))
         (range (loop for k from 0 below 1 by step-adj collect k))
         (range (if (< (car (last range)) 1)
                  (append range '(1))
                  range)))
    (mapcar (lambda (r)
              (cl-transforms-stamped:make-pose-stamped (cl-transforms-stamped:frame-id start) (cl-transforms-stamped:stamp start)
                (cl-transforms:v+ (cl-transforms:origin start)
                                  (cl-transforms:v* seg r))
                (cl-transforms:orientation start)))
            range)))

(defun perform-cut-skeleton (cut-skeleton-wrapper tool-name object-name maneuver-arm aux-arm tf-transformer arm-capmap slices-marker &optional (init-sup-man-count 0))
;; Reset skeleton markers
  (reset-skeleton-markers)
;; Only do something when there's something actually in the cut-skeleton
  (when (cut-skeleton cut-skeleton-wrapper)
;; Update the cut skeleton's transforms
    (let* ((object-loc (cl-tf:lookup-transform tf-transformer "map" object-name))
           (skeleton-to-tool (get-skeleton-to-tool tool-name)))
      (setf (plan-to-environment-transform cut-skeleton-wrapper) object-loc)
      (setf (skeleton-to-tool-transform cut-skeleton-wrapper) skeleton-to-tool))
;; Update markers
    (update-markers cut-skeleton-wrapper object-name tool-name slices-marker tf-transformer)
;; Do auxiliary maneuvers to assist cutting
    (let* ((arm-capmap-loc (cl-tf:lookup-transform tf-transformer "map" "torso_lift_link"))
           (object-loc (cl-tf:lookup-transform tf-transformer "map" object-name))
           (pose-suggestions (suggest-placement-transform cut-skeleton-wrapper maneuver-arm arm-capmap-loc :reachability-map arm-capmap))
           (suggested-transform (get-valid-pose-suggestion pose-suggestions))
           (suggested-transform-viz (cl-transforms:make-transform (cl-transforms:v+ (cl-transforms:translation suggested-transform)
                                                                                    (cl-transforms:make-3d-vector 0 0 0.05))
                                                                  (cl-transforms:rotation suggested-transform))))
      (place-object-group-markers "pose-suggestion" object-name slices-marker :location suggested-transform-viz :alpha 0.35)
      (roslisp:wait-duration 1)
      (place-reachmap-markers pose-suggestions "map" object-loc)
      (setf init-sup-man-count
            (+ (reposition-object object-name object-loc arm-capmap-loc
                                  suggested-transform maneuver-arm aux-arm tf-transformer)
               init-sup-man-count))
      (setf (plan-to-environment-transform cut-skeleton-wrapper)
            (cl-tf:lookup-transform tf-transformer "map" object-name))
      (move-arm-poses aux-arm (get-park-pose aux-arm)))
;; Follow the skeleton segment: first, prepare the poses to send for the arm
    (let* ((segment (get-current-segment cut-skeleton-wrapper))
           (seg-prestart (segment-prestart segment))
           (seg-prestart (cl-transforms-stamped:make-pose-stamped "map" 0 (cl-transforms:translation seg-prestart) (cl-transforms:rotation seg-prestart)))
           (seg-start (segment-start segment))
           (seg-start (cl-transforms-stamped:make-pose-stamped "map" 0 (cl-transforms:translation seg-start) (cl-transforms:rotation seg-start)))
           (seg-end (segment-end segment))
           (seg-end (cl-transforms-stamped:make-pose-stamped "map" 0 (cl-transforms:translation seg-end) (cl-transforms:rotation seg-end)))
           (seg-postend (segment-postend segment))
           (seg-postend (cl-transforms-stamped:make-pose-stamped "map" 0 (cl-transforms:translation seg-postend) (cl-transforms:rotation seg-postend)))
           (seg-waypoints (get-seg-waypoints seg-start seg-end 0.03)))
;; Follow the current skeleton segment; do it a few times, for style
      (move-arm-poses maneuver-arm 
                      (append (list seg-prestart seg-start)
                              seg-waypoints
                              (reverse seg-waypoints)
                              seg-waypoints
                              (list seg-postend (get-park-pose maneuver-arm))))
;; Finally, recur to do the remaining skeleton segments
      (pop-skeleton-segment cut-skeleton-wrapper)
      (setf init-sup-man-count
            (perform-cut-skeleton cut-skeleton-wrapper tool-name object-name 
                                  maneuver-arm aux-arm tf-transformer arm-capmap slices-marker
                                  init-sup-man-count))))
  init-sup-man-count)

(cpl-impl:def-cram-function perform-cut (object-name tool-name cut-skeleton-wrapper amount slices-marker)
  (cram-beliefstate:enable-logging t)
  (cram-beliefstate::start-new-experiment)
  (cram-beliefstate:set-metadata :robot "PR2" :creator "IAI"
                                 :experiment "Cut with assistive maneuvers"
                                 :description (format nil "Perform ~a cuts (so as to get ~a slices) on the ~a with the ~a." (length (cut-skeleton cut-skeleton-wrapper)) amount object-name tool-name))
  (cram-beliefstate::set-experiment-meta-data "performedInMap" "http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j" :type :resource :ignore-namespace t)
  (let* ((log-node-id (cram-beliefstate:start-node "SLICING" nil))
         (tf-transformer cram-moveit::*transformer*)
         (cut-skeleton-wrapper (make-instance 'cut-skeleton-wrapper
                                              :skeleton-to-tool-transform (get-skeleton-to-tool tool-name)
                                              :plan-to-environment-transform (cl-tf:lookup-transform tf-transformer "map" object-name)
                                              :cut-skeleton (mapcar (lambda (seg)
                                                                      (make-instance 'skeleton-segment
                                                                                     :segment-start (segment-start seg)
                                                                                     :segment-end (segment-end seg)
                                                                                     :segment-prestart (segment-prestart seg)
                                                                                     :segment-postend (segment-postend seg)))
                                                                    (cut-skeleton cut-skeleton-wrapper))))
         (desired-base-pose (get-desired-base-pose object-name tf-transformer)))
    (cram-beliefstate::annotate-resource "toolUsed" (get-object-semmap-name tool-name) "knowrob")
    (cram-beliefstate::annotate-resource "objectActedOn" (get-object-semmap-name object-name) "knowrob")
    (cram-beliefstate::annotate-resource "numberOfSlices" amount "knowrob")
    (cram-beliefstate::annotate-resource "numberOfCuts" (length (cut-skeleton cut-skeleton-wrapper)))
    (place-object-group-markers "object-markers" object-name slices-marker)
    (setup-pr2 desired-base-pose)
    (let* ((tool-grabbing-arm (get-tool-grabbing-arm object-name tool-name tf-transformer))
           (maneuver-arm (get-maneuver-arm object-name tool-name tf-transformer))
           (arm-capmap (get-arm-capmap maneuver-arm))
           (supportive-maneuvers 0)
           (aux-arm (get-aux-arm object-name tool-name tf-transformer)))
      (handover-tool tool-name object-name tool-grabbing-arm maneuver-arm tf-transformer nil nil)
      (setf supportive-maneuvers
            (perform-cut-skeleton cut-skeleton-wrapper tool-name object-name maneuver-arm aux-arm tf-transformer arm-capmap slices-marker))
      (cram-beliefstate::annotate-resource "numberOfSupportiveManeuvers" supportive-maneuvers "knowrob")
      (handover-tool tool-name object-name maneuver-arm tool-grabbing-arm tf-transformer t t)
      (cram-beliefstate:stop-node log-node-id)))
  (let* ((date-time (multiple-value-bind (second minute hour date month year) (get-decoded-time) (format nil "~d-~2,'0d-~2,'0d--~2,'0d:~2,'0d:~2,'0d" year month date hour minute second)))
         (file-name (format nil "perform-cut-~a" date-time))
         (dot-name (format nil "~a.dot" file-name))
         (owl-name (format nil "~a.owl" file-name)))
    (cram-beliefstate:extract-dot-file dot-name)
    (cram-beliefstate:extract-owl-file owl-name)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; action_core: Cutting
;; action_roles: ['action_verb', 'amount', 'obj_to_be_cut', 'unit', 'utensil']
;; cram_plan: "(cut (an object (type {obj_to_be_cut}){obj_to_be_cut_props})(into (amount (a quantity (type {unit})(number {amount}))))(with (an object (type {utensil}){utensil_props})))"

;; action_core: Pouring
;; action_roles: ['stuff','goal','action_verb','unit','amount']
;; required_action_roles: ['stuff','goal','action_verb']
;; cram_plan: "(pour-from-container
;;               (from (an object  (type container.n.01) (contains (some stuff (type {stuff}){stuff_props})))) (a quantity (type {unit})(number {amount})) (to (an object  (type {goal}){goal_props})))"

(defun cleanup-action-roles (action-roles)
  (let* ((action-roles (mapcar (lambda (action-role)
                                 (roslisp:with-fields ((role-name role_name) (role-value role_value)) action-role
                                   (let* ((point-pos (position #\. role-value))
                                          (role-value (if point-pos
                                                          (subseq role-value 0 point-pos)
                                                          role-value)))
                                     (list role-name role-value))))
                               action-roles)))
    action-roles))

(defun get-amount (amount)
  (cond
    ((not amount) 4)
    ((equal (string-downcase amount) "unknown") 1)
    ((equal amount "zero") 0)
    ((equal amount "one") 1)
    ((equal amount "two") 2)
    ((equal amount "three") 3)
    ((equal amount "four") 4)
    ((equal amount "five") 5)
    ((equal amount "six") 6)
    ((equal amount "seven") 7)
    ((equal amount "eight") 8)
    ((equal amount "nine") 9)
    (t amount)))

(defun objects-in-map (object-name tool-name)
  (let* ((known-objects '(("pizza_plate" 1) ("pizza_cutter" 2) ("bread" 3) ("knife" 4))))
    (and (assoc object-name known-objects :test #'equal) (assoc tool-name known-objects :test #'equal))))

(defun get-yaw (loc)
  (let* ((x-dir (cl-transforms:make-3d-vector 1 0 0))
         (x-dir (cl-transforms:rotate (cl-transforms:rotation loc) x-dir)))
    (atan (cl-transforms:y x-dir)
          (cl-transforms:x x-dir))))

(defun sanity-check (amount object-name)
  (cond
    ((equal object-name "pizza_plate")
      (let* ((amount (if (< 1 amount)
                       amount
                       1))
             (amount (if (< amount 7)
                       amount
                       7))
             (amount (floor amount)))
        amount))
    ((equal object-name "bread")
      (let* ((amount (if (< 0 amount)
                       amount
                       0))
             (amount (if (< amount 10)
                       amount
                       10))
             (amount (floor amount)))
        amount))))

(defun get-pizza-cut-skeleton-wrapper (amount)
  (let* ((transformer cram-moveit::*transformer*)
         (pizza-loc (cl-tf:lookup-transform transformer "map" "pizza_plate"))
         (robot-at-pizza-loc (get-desired-base-pose "pizza_plate" transformer))
         (amount (sanity-check amount "pizza_plate"))
         (need-cuts (< 1 amount))
         (is-even (evenp amount))
         (angle-increment (/ (* pi 2) amount))
         (amount (if is-even
                   (/ amount 2)
                   amount))
         (indices (if need-cuts
                    (alexandria:iota amount)
                    nil))
         (start-base (cl-transforms:make-3d-vector *plate-radius* 0 0.02))
         (end-base (if is-even
                     (cl-transforms:make-3d-vector (- 0 *plate-radius*) 0 0.02)
                     (cl-transforms:make-3d-vector 0 0 0.02)))
         (robot-angle (get-yaw (cl-transforms:make-transform (cl-transforms:origin robot-at-pizza-loc)
                                                             (cl-transforms:orientation robot-at-pizza-loc))))
         (plate-angle (get-yaw pizza-loc))
         (convenience-angle (+ (- 0 (* (/ pi 4) 3)) (- 0 plate-angle) robot-angle))
         (convenience-rotation (cl-transforms:euler->quaternion :az convenience-angle))
         (segments (mapcar (lambda (k)
                             (let* ((angle (* k angle-increment))
                                    (z-rotation (cl-transforms:euler->quaternion :az angle))
                                    (start (cl-transforms:rotate z-rotation start-base))
                                    (end (cl-transforms:rotate z-rotation end-base)))
                               (vector-pair->skeleton-segment (cl-transforms:rotate convenience-rotation start)
                                                              (cl-transforms:rotate convenience-rotation end))))
                           indices)))
    (make-instance 'cut-skeleton-wrapper
                   :cut-skeleton segments
                   :skeleton-to-tool-transform (cl-transforms:make-identity-transform)
                   :plan-to-environment-transform (cl-transforms:make-identity-transform))))

(defun get-bread-cut-skeleton-wrapper (amount)
  (let* ((amount (sanity-check amount "bread"))
         (slice-thickness 0.015)
         (indices (alexandria:iota amount))
         (start-base (cl-transforms:make-3d-vector (* *bread-length* 0.5) (- 0 (* *bread-width* 0.5)) 0.03))
         (end-base (cl-transforms:make-3d-vector (* *bread-length* 0.5) (* *bread-width* 0.5) 0.03))
         (disp (cl-transforms:make-3d-vector (- 0 slice-thickness) 0 0))
         (segments (mapcar (lambda (k)
                             (let* ((k (+ k 1))
                                    (disp (cl-transforms:v* disp k))
                                    (start (cl-transforms:v+ start-base disp))
                                    (end (cl-transforms:v+ end-base disp)))
                               (vector-pair->skeleton-segment start end)))
                           indices)))
    (make-instance 'cut-skeleton-wrapper
                   :cut-skeleton segments
                   :skeleton-to-tool-transform (cl-transforms:make-identity-transform)
                   :plan-to-environment-transform (cl-transforms:make-identity-transform))))

(defun get-cut-skeleton-wrapper (object-name amount)
  (cond
    ((equal object-name "pizza_plate")
      (get-pizza-cut-skeleton-wrapper amount))
    ((equal object-name "bread")
      (get-bread-cut-skeleton-wrapper amount))))

(defun perform-cut-get-args (&rest action-roles)
  (let* ((action-roles (cleanup-action-roles action-roles))
         (object-name-input (car (cdr (assoc "obj_to_be_cut" action-roles :test #'equal))))
         (object-name (if (equal object-name-input "pizza")
                        "pizza_plate"
                        object-name-input))
         (object-name (if (member object-name '("italian_bread" "baguette" "baguet" "ciabatta") :test #'equal)
                        "bread"
                        object-name))
         ;;(tool-name (car (cdr (assoc "utensil" action-roles :test #'equal))))
         (tool-name (cond
                      ((equal object-name "pizza_plate") "pizza_cutter")
                      ((equal object-name "bread") "knife")))
         (unit (car (cdr (assoc "unit" action-roles :test #'equal))))
         (amount (get-amount (car (cdr (assoc "amount" action-roles :test #'equal)))))
         (amount (if (typep amount 'string) (parse-integer amount :junk-allowed t) amount))
         (should-run-plan (objects-in-map object-name tool-name))
         (amount (when should-run-plan
                   (sanity-check amount object-name)))
         (cut-skeleton-wrapper (when should-run-plan
                                 (get-cut-skeleton-wrapper object-name amount)))
         (msg (if should-run-plan
                (format nil "perform-cut using ~a to cut ~a ~a out of ~a~%" tool-name amount unit object-name-input)
                (format nil "Object/tool pair (~a ~a) contains an unrecognized object.~%" object-name tool-name)))
         (plan-string (if should-run-plan
                        (format nil "(perform-cut ~a ~a cut-skeleton-wrapper)~%" object-name tool-name)
                        (format nil "")))
         (args (when should-run-plan
                 (list object-name tool-name cut-skeleton-wrapper amount nil))))
    (values (if should-run-plan 0 -1)
            args
            msg
            plan-string)))

(cpl-impl:def-cram-function con-test (&rest args)
  (format t "Triggered a plan with args ~a~%" args))

(defun cut-test-get-args (&rest args)
  (declare (ignore args))
  (let* ((should-run-plan t)
         (message "Will now cut out a particular slice of pizza.")
;;;;;; !!!!!!!! Slices marker should not be nil, but a (mesh-path angle) pair
         (args (list "pizza_plate" "pizza_cutter" *cut-skeleton-wrapper* 1 nil))
         (plan-string "(perform-cut pizza_plate pizza_cutter cut-skeleton-wrapper slices-marker)"))
    (values (if should-run-plan 0 -1)
            args
            message
            plan-string)))

(defun pouring-get-args (&rest args)
  (declare (ignore args))
  (let* ((should-run-plan nil))
    (values (if should-run-plan 0 -1)
            nil
            "Not implemented yet."
            "")))

(cpl-impl:def-cram-function pouring-top-level ()
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun get-init-pose (object-name tf-transformer)
  (let* ((transform (cl-tf:lookup-transform tf-transformer "map" object-name)))
    (roslisp:make-message "geometry_msgs/Pose"
                          :position (roslisp:make-message "geometry_msgs/Point"
                                                          :x (cl-tf:x (cl-tf:translation transform))
                                                          :y (cl-tf:y (cl-tf:translation transform))
                                                          :z (cl-tf:z (cl-tf:translation transform)))
                          :orientation (roslisp:make-message "geometry_msgs/Quaternion"
                                                             :x (cl-tf:x (cl-tf:rotation transform))
                                                             :y (cl-tf:y (cl-tf:rotation transform))
                                                             :z (cl-tf:z (cl-tf:rotation transform))
                                                             :w (cl-tf:w (cl-tf:rotation transform))))))

(defun set-gazebo-object-pose (model-name model-pose)
  (roslisp:call-service "/gazebo/set_model_state" 'gazebo_msgs-srv:SetModelState
                        :model_state (roslisp:make-message "gazebo_msgs/ModelState"
                                                           :model_name model-name
                                                           :pose model-pose
                                                           :reference_frame "map")))

(defun cancel-function ()
  (detach-model "pr2" (mot-man:eef-link-name :left) "pizza_plate" "pizza_plate")
  (detach-model "pr2" (mot-man:eef-link-name :right)  "pizza_plate" "pizza_plate")
  (detach-model "pr2" (mot-man:eef-link-name :left) "pizza_cutter" "pizza_cutter")
  (detach-model "pr2" (mot-man:eef-link-name :right)  "pizza_cutter" "pizza_cutter")
  (detach-model "pr2" (mot-man:eef-link-name :left) "bread" "bread")
  (detach-model "pr2" (mot-man:eef-link-name :right) "bread" "bread")
  (detach-model "pr2" (mot-man:eef-link-name :left)  "knife" "knife")
  (detach-model "pr2" (mot-man:eef-link-name :right)  "knife" "knife")
  (set-gazebo-object-pose "pizza-plate" *initial-pose-pizza*)
  (set-gazebo-object-pose "pizza-cutter" *initial-pose-pizza-cutter*)
  (set-gazebo-object-pose "bread" *initial-pose-bread*)
  (set-gazebo-object-pose "knife" *initial-pose-knife*)
  (attach-model "IAI_kitchen" "room_link" "pizza_plate" "pizza_plate")
  (attach-model "IAI_kitchen" "room_link" "pizza_cutter" "pizza_cutter")
  (attach-model "IAI_kitchen" "room_link" "bread" "bread")
  (attach-model "IAI_kitchen" "room_link" "knife" "knife")
  (move-arms-up))

(defparameter *pracsimserver-plan-matchings*
              (list (cons "Cutting" (list #'perform-cut #'perform-cut-get-args))
                    (cons "Pouring" (list #'pouring-top-level #'pouring-get-args))
                    (cons "Cut-Test" (list #'perform-cut #'cut-test-get-args))))

(defun start-scenario ()
  (roslisp-utilities:startup-ros)
  (semantic-map-collision-environment:publish-semantic-map-collision-objects)
  (setf *initial-pose-pizza* (get-init-pose "pizza_plate" cram-moveit::*transformer*))
  (setf *initial-pose-pizza-cutter* (get-init-pose "pizza_cutter" cram-moveit::*transformer*))
  (setf *initial-pose-bread* (get-init-pose "bread" cram-moveit::*transformer*))
  (setf *initial-pose-knife* (get-init-pose "knife" cram-moveit::*transformer*))
  (prac2cram:prac2cram-server *pracsimserver-plan-matchings* #'cancel-function)
  ;;(perform-cut "pizza_plate" "pizza_cutter" pizza-ninja::*cut-skeleton-wrapper* nil)
  (let* ((a 1) (b 1) (s 1))
    (loop
      (let ((c (rem (+ a b) 97)))
        (roslisp:wait-duration 1)
        (format t "Tick-tock ~a: ~a.~%" s c)
        (setf s (+ s 1))
        (setf s (if (<= 100 s) 0 s))
        (setf a b)
        (setf b c)))))

