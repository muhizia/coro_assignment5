(in-package :pp-tut)
(defparameter *base-pose-near-table*
  (make-pose "map" '((-1.447 -0.15 0.0) (0.0 0.0 -0.7071 0.7071))))
 
(defparameter *downward-look-coordinate*
  (make-pose "base_footprint" '((0.65335 0.076 0.758) (0 0 0 1))))
 
(defparameter *base-pose-near-counter*
  (make-pose "map" '((-0.15 2 0) (0 0 -1 0))))
 
(defparameter *final-object-destination*
  (make-pose "map" '((-0.8 2 0.9) (0 0 0 1))))

(defparameter *left-downward-look-coordinate*
  (make-pose "base_footprint" '((0.65335 0.76 0.758) (0 0 0 1))))
 
(defparameter *right-downward-look-coordinate*
  (make-pose "base_footprint" '((0.65335 -0.76 0.758) (0 0 0 1))))

; more base poses
(defparameter *new-base-pose-near-counter*
  (make-pose "map" '((-0.15 2 0) (0 0 -1 0))))
 
 
; (defparameter *base-pose-near-counter*
;   (make-pose "map" '((-0.15 2 0) (0 0 -1 0))))
 
; (defparameter *final-object-destination*
;   (make-pose "map" '((-0.8 2 0.9) (0 0 0 1))))

(defun find-object (?object-type)
  (let* ((possible-look-directions `(,*downward-look-coordinate*
                                     ,*left-downward-look-coordinate*
                                     ,*right-downward-look-coordinate*))
         (?looking-direction (first possible-look-directions)))
    (setf possible-look-directions (rest possible-look-directions))
    ;; Look towards the first direction
    (perform (an action
                 (type looking)
                 (target (a location 
                            (pose ?looking-direction)))))
 
    ;; perception-object-not-found is the error that we get when the robot cannot find the object.
    ;; Now we're wrapping it in a failure handling clause to handle it
    (handle-failure perception-object-not-found
        ;; Try the action
        ((perform (an action
                      (type detecting)
                      (object (an object 
                                  (type ?object-type))))))
 
      ;; If the action fails, try the following:
      ;; try different look directions until there is none left.
      (when possible-look-directions
        (print "Perception error happened! Turning head.")
        ;; Resetting the head to look forward before turning again
        (perform (an action
                     (type looking) 
                     (direction forward)))
        (setf ?looking-direction (first possible-look-directions))
        (setf possible-look-directions (rest possible-look-directions))
        (perform (an action 
                     (type looking)
                     (target (a location
                                (pose ?looking-direction)))))
        ;; This statement retries the action again
        (cpl:retry))
      ;; If everything else fails, error out
      ;; Reset the neck before erroring out
      (perform (an action
                   (type looking)
                   (direction forward)))      
      (cpl:fail 'object-nowhere-to-be-found))))

(defun pick-up-object (?perceived-object ?grasping-arm)
  (let* ((?possible-grasps '(:left-side :right-side :front :back))
         (?grasp (first ?possible-grasps)))
    (setf ?possible-grasps (rest ?possible-grasps))
 
    (cpl:with-retry-counters ((arm-change-retry 1))
      ;; Outer handle failure handling arm change
      (handle-failure object-unreachable
          ;; Iner handle-failure handling grasp change
          ((handle-failure (or manipulation-pose-unreachable gripper-closed-completely)
               ;; Try to perform the pick up
               ((perform (an action
                             (type picking-up)
                             (arm ?grasping-arm)
                             (grasp ?grasp)
                             (object ?perceived-object))))
             ;; When pick-up fails this block gets executed
             (format t "Grasp failed! Error: ~a~%Grasp: ~a~%Arm: ~a~%"
                     e ?grasp ?grasping-arm)
             ;; Checks if we have any possible grasps left.
             ;; If yes, then the block nested to it gets executed, which will
             ;; set the grasp that is used to the new value and trigger retry
             (when (first ?possible-grasps)
               (format t "Retyring! Trying to grasp from ~a using ~a arm"
                       ?grasp ?grasping-arm)
               (setf ?grasp (first ?possible-grasps))
               (setf ?possible-grasps (rest ?possible-grasps))
               (park-arms)
               (cpl:retry))
             ;; This will get executed when there are no more elements in the 
             ;; ?possible-grasps list. We print the error message and throw a new error
             ;; which will be caught by the outer handle-failure
             (print  "No more grasp retries left :( :) ")
             (cpl:fail 'object-unreachable)))
 
        ;; This is the failure management of the outer handle-failure call
        ;; It changes the arm that is used to grasp
        (print "Manipulation failed!!")
        (print e)
 
        ;; Here we use the retry counter we defined. The value is decremented automatically
        (cpl:do-retry arm-change-retry
          ;; if the current grasping arm is right set left, else set right
          (setf ?grasping-arm (if (eq ?grasping-arm :right) 
                                  :left
                                  :right))
          (cpl:retry))
        ;; When all retries are exhausted print the error message.
        (print "No more arm change retries left :("))))
  ?grasping-arm)

(defun move-bottle (bottle-spawn-pose)
  (spawn-object bottle-spawn-pose)
  (with-simulated-robot
    ; (let ((?navigation-goal *base-pose-near-table*))
    ;   (cpl:par
    ;     ;; Moving the robot near the table.
    ;     (perform (an action
    ;                  (type going)
    ;                  (target (a location 
    ;                             (pose ?navigation-goal)))))
    ;     (perform (a motion
    ;                 (type moving-torso) 
    ;                 (joint-angle 0.3)))
    ;     (park-arms))
    (find-object-with-base-movement)
    (perform (a motion
                    (type moving-torso) 
                    (joint-angle 0.3)))
    (park-arms))
    )

; find-object
(defun find-object-with-base-movement ()
  (let* ((possible-look-locations `(,*new-base-pose-near-counter*
                                     ,*base-pose-near-table*))
         (?looking-location (first possible-look-locations)))
    (setf possible-look-locations (rest possible-look-locations))
    ;; Look towards the first location
    (perform (an action
                     (type going)
                     (target (a location 
                                (pose ?looking-location)))))
    (perceive-bottle)
    ;; perception-object-not-found is the error that we get when the robot cannot find the object.
    ;; Now we're wrapping it in a failure handling clause to handle it
    (handle-failure perception-object-not-found
        ;; Try the action
        ;; Calling the function to find the object
 
      ;; If the action fails, try the following:
      ;; try different look directions until there is none left.
      (when possible-look-locations
        (print "error happened! checking in other locations.")
        ;; Resetting the head to look forward before turning again
        (setf ?looking-location (first possible-look-locations))
        (setf possible-look-locations (rest possible-look-locations))
        (perform (an action
                     (type going)
                     (target (a location 
                                (pose ?looking-location)))))
        (perceive-bottle)
        ;; This statement retries the action again
        (cpl:retry))
      ;; If everything else fails, error out
      ;; Reset the neck before erroring out     
      (cpl:fail 'object-nowhere-to-be-found))
      ))

    (defun perceive-bottle()
      (let ((?perceived-bottle (find-object :bottle))
           (?grasping-arm :right))
      ;; We update the value of ?grasping-arm according to what the method used
      (setf ?grasping-arm (pick-up-object ?perceived-bottle ?grasping-arm))
      (park-arm ?grasping-arm)
      ;; Moving the robot near the counter.
      (let ((?nav-goal *base-pose-near-counter*))
          (perform (an action
                      (type going)
                      (target (a location 
                                  (pose ?nav-goal))))))
      ;; Setting the object down on the counter
      (let ((?drop-pose *final-object-destination*))
          (perform (an action
                      (type placing)
                      (arm ?grasping-arm)
                      (object ?perceived-bottle)
                      (target (a location 
                                  (pose ?drop-pose))))))
      (park-arm ?grasping-arm)))