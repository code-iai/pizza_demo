; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem pizza-ninja
  :name "pizza-ninja"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Uses giskard to perform cutting actions."
  :depends-on (:cl-transforms-stamped
               :alexandria
               :cl-tf
               :meshproc_msgs-srv
               :attache_msgs-srv
               :visualization_msgs-msg
               ;;:cutplan-srv
               ;;:cutplan
               :cl-giskard
               :ros-load-manifest
               :roslisp-utilities
               :roslisp
               :ros-load-manifest
               :cram-moveit-manager
               :prac2cram
               ;;:pr2-manipulation-process-module
               ;;
               cram-tf
               cram-process-modules
               actionlib
               roslisp-utilities
               cram-prolog
               cram-plan-failures
               cram-plan-occasions-events
               cram-occasions-events
               cram-pr2-description
               pr2_controllers_msgs-msg
               pr2_msgs-msg
               pr2_msgs-srv
               trivial-garbage
               std_srvs-srv
               alexandria
               semantic-map-collision-environment
               pr2_mechanism_msgs-srv
               cram-motion-manager
               cram-moveit
               cram-beliefstate
               ;;
               :pr2-navigation-process-module
               :pr2-reachability-costmap)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "pizza-ninja" :depends-on ("package"))
             (:file "markers" :depends-on ("package"))
             (:file "demo" :depends-on ("package" "pizza-ninja" "markers"))))))
