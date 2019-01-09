; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem pizza-ninja
  :name "pizza-ninja"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Uses giskard to perform cutting actions."
  :depends-on (:alexandria
               :visualization_msgs-msg
               :ros-load-manifest
               :roslisp-utilities
               :roslisp
               :prac2cram
               :cram-tf
               :actionlib
               :cram-prolog
               :cram-pr2-description
               :trivial-garbage
               :semantic-map-collision-environment
               :cram-executive
               :cram-pr2-pick-place-plans
               :cram-pr2-process-modules
               :pr2-reachability-costmap)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "pizza-ninja" :depends-on ("package"))
             (:file "markers" :depends-on ("package" "pizza-ninja"))
             (:file "demo" :depends-on ("package" "markers"))))))
