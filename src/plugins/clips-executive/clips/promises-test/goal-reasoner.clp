;---------------------------------------------------------------------------
;  goal-reasoner.clp - Goal reasoning for RCLL domain
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;

(defglobal
  ; Number of retrying enter-field
  ; until succeeding it manually
  ?*ENTER-FIELD-RETRIES* = 1
  ?*MAX-RETRIES-PICK* = 2
  ?*MAX-RETRIES-PUT-SLIDE* = 2
  ?*GOAL-MAX-TRIES* = 3

  ?*SALIENCE-GOAL-FORMULATE* = 500
  ?*SALIENCE-GOAL-REJECT* = 400
  ?*SALIENCE-GOAL-EXPAND* = 300
  ?*SALIENCE-GOAL-SELECT* = 200
  ; PRE-EVALUATE rules should do additional steps in EVALUATION but must not
  ; set the goal to EVALUATED
  ?*SALIENCE-GOAL-PRE-EVALUATE* = 1
  ; common evaluate rules should have
  ;   lower salience than case specific ones
  ?*SALIENCE-GOAL-EVALUATE-GENERIC* = -1
)

(deffunction requires-subgoal (?goal-type)
  (return (or (eq ?goal-type RUN-ONE-OF-SUBGOALS)
              (eq ?goal-type RUN-ENDLESS)))
)


(deffunction production-goal (?goal-class)
  (return (or
            (eq ?goal-class FILL-CONTAINER)
            (eq ?goal-class DELIVER)
            (eq ?goal-class DELIVER-XENONITE)
            (eq ?goal-class CLEAN-MACHINE)
            (eq ?goal-class START-MACHINE)
          ))
)

(deffunction goal-tree-assert-run-endless (?class ?id ?host $?fact-addresses)
        (bind ?goal (assert (goal (id ?id) (class ?class) (type MAINTAIN)
                            (sub-type RUN-ENDLESS) (params frequency 1)
                            (meta last-formulated (now) host ?host))))
        (foreach ?f ?fact-addresses
                (goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
        (return ?goal)
)

(deffunction goal-tree-assert-run-one-test (?class ?host $?fact-addresses)
	(bind ?id (sym-cat RUN-ONE- ?class - (gensym*)))
	(bind ?goal (assert (goal (id ?id) (class ?class) (sub-type RUN-ONE-OF-SUBGOALS) (meta host ?host))))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-subtree (?id $?fact-addresses)
        (foreach ?f ?fact-addresses
                (goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
)

(defrule goal-reasoner-create-production-maintain
" The parent production goal. Allows formulation of
  production goals only if the proper game state selected
  and the domain got loaded. Other production goals are
  formulated as sub-goals of this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  =>
  (goal-tree-assert-run-endless PRODUCTION-MAINTAIN PRODUCTION-MAINTAIN-WallE WallE)
  (goal-tree-assert-run-endless PRODUCTION-MAINTAIN PRODUCTION-MAINTAIN-Eve Eve)
  (goal-tree-assert-run-endless PRODUCTION-MAINTAIN PRODUCTION-MAINTAIN-R2D2 R2D2)
  (goal-tree-assert-run-endless PRODUCTION-MAINTAIN PRODUCTION-MAINTAIN-Arnie Arnie)
)

(defrule goal-reasoner-select-root
"  Select all root goals (having no parent) in order to expand them."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type ACHIEVE|MAINTAIN) (sub-type ~nil) (id ?goal-id) (mode FORMULATED))
=>
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-selection-assert-waiting
  ;there is a robot that is not yet waiting
  (domain-object (name ?r) (type robot))
  (not (waiting ?r))

  ;there is an executable goal for the robot
  (goal-class (class ?class) (id ?cid) (sub-type ?subtype) (lookahead-time ?lt))
  (pddl-formula (part-of ?cid) (id ?formula-id))
  (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied ?sat) (promised-from ?from) (grounding ?grounding-id))
  (pddl-grounding (id ?grounding-id) (param-values $? ?r $?))

  (time ?now ?mills)
  (test (sat-or-promised ?sat ?now ?from ?lt))

  ;there is not a production goal strand
  (not (goal (class PRODUCTION-RUN-ONE) (meta $? host ?r $?)))
  =>
  (assert (waiting ?r))
)

(defrule goal-reasoner-selection-assert-select
  (waiting ?r)
  (not (selecting ? ?))
  (not (tried ?r))
  (time ?now ?mills)
  =>
  (assert (selecting ?r ?now))
)

(defrule goal-reasoner-selection-retract-select
  ?s <- (selecting ?r ?time)
  (time ?now ?mills)
  (test (> ?now ?time))
  =>
  (retract ?s)
  (assert (tried ?r))
)

(defrule goal-reasoner-selection-retract-waiting-subgoal-selected
  ?w <- (waiting ?r)
  (goal (class PRODUCTION-RUN-ONE) (mode DISPATCHED) (meta $? host ?r $?))
  =>
  (retract ?w)
)

(defrule goal-rasoner-selection-retract-select-subgoal-selected
  ?s <- (selecting ?r ?)
  (goal (class PRODUCTION-RUN-ONE) (mode DISPATCHED) (meta $? host ?r $?))
  =>
  (retract ?s)
)

(defrule goal-reasoner-selection-restart-selection
  (not
    (and
      (waiting ?r)
      (not (tried ?r))
    )
  )
  (not (selecting ?r ?))
  =>
  (do-for-all-facts ((?t tried)) TRUE
    (retract ?t)
  )
  (do-for-all-facts ((?w waiting)) TRUE
    (retract ?w)
  )
)

(defrule goal-reasoner-selection-assert-run-one-subetree
  (goal (class PRODUCTION-MAINTAIN) (mode SELECTED) (meta $? host ?host) (id ?id))
  (not (goal (parent ?id) (mode FORMULATED|SELECTED|EXPANDED|DISPATCHED)))
  (selecting ?host ?)
  =>
  (bind ?g (goal-tree-assert-run-one-test PRODUCTION-RUN-ONE ?host))
  (modify ?g (parent ?id))
)

(defrule goal-reasoner-selection-retract-run-one-subtree-no-child
  (domain-object (name ?host) (type robot))
  ?p <- (goal (id ?grandparent) (class PRODUCTION-MAINTAIN))
  ?g <- (goal (id ?parent) (parent ?grandparent) (class PRODUCTION-RUN-ONE) (meta host ?host))
  (not (goal (parent ?parent)))
  (not (selecting ?host ?))
  =>
  (retract ?g)
  (modify ?p (mode FORMULATED))
)


(defrule goal-reasoner-expand-goal-with-sub-type
" Expand a goal with sub-type, if it has a child."
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?p <- (goal (id ?parent-id) (type ACHIEVE|MAINTAIN)
              (sub-type ?sub-type&:(requires-subgoal ?sub-type)) (mode SELECTED))
  ?g <- (goal (parent ?parent-id) (mode FORMULATED))
=>
  (modify ?p (mode EXPANDED))
)

(defrule goal-reasoner-evaluate-common
" Finally set a finished goal to evaluated.
  All pre evaluation steps should have been executed, enforced by the higher priority
"
  (declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome))
=>
  ;(printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
  ;  "') has been completed, Evaluating" crlf)
  (modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-evaluate-production-maintain
  "Clean up all rs-fill-priorities facts when the production maintenance goal
   fails."
  ?g <- (goal (id ?goal-id) (class PRODUCTION-MAINTAIN) (parent nil)
              (mode FINISHED) (outcome ?outcome))
  =>
  (printout t "Goal '" ?goal-id "' has been " ?outcome ", evaluating" crlf)
  (do-for-all-facts ((?prio wm-fact)) (wm-key-prefix ?prio:key (create$ evaluated fact rs-fill-priority))
   (retract ?prio))
  (modify ?g (mode EVALUATED))
)


; ================================= Goal Clean up ============================

(defrule goal-reasoner-remove-simple-goals-one-selected
  (goal (id ?parent) (class PRODUCTION-RUN-ONE))
  (goal (parent ?parent) (mode ~FORMULATED))
  ?g <- (goal (type ACHIEVE) (mode FORMULATED) (parent ?parent))
  =>
  (retract ?g)
)


(defrule goal-reasoner-remove-retracted-goal-common
" Remove a retracted goal if it has no child (anymore).
  Goal trees are retracted recursively from bottom to top. This has to be done
  with low priority to avoid races with the sub-type goal lifecycle.
"
  (declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
  ?g <- (goal (id ?goal-id)
        (mode RETRACTED) (acquired-resources))
  (not (goal (parent ?goal-id)))
=>
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (and (eq ?a:plan-id ?p:id) (eq ?a:goal-id ?goal-id))
      (retract ?a)
    )
    (retract ?p)
  )
  (retract ?g)
)

(defrule goal-reasoner-reject-production-tree-goals-other-goal-dispatched
" Retract all formulated sub-goal of the production tree once a production leaf
  goal is dispatched.
"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?goal) (parent ?parent) (type ACHIEVE)
        (sub-type ?sub-type) (class ?class&:(production-goal ?class))
        (mode FORMULATED))
  (goal (id ?some-leaf) (parent ?parent) (class ?some-class&:(production-goal ?some-class))
        (sub-type SIMPLE) (mode DISPATCHED))
=>
  (delayed-do-for-all-facts ((?g goal))
    (and (eq ?g:mode FORMULATED) (production-goal ?g:class) (eq ?g:parent ?parent))
    (modify ?g (mode RETRACTED) (outcome REJECTED))
  )
)

(defrule goal-reasoner-error-goal-without-sub-type-detected
" This goal reasoner only deals with goals that have a sub-type. Other goals
  are not supported.
"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?goal) (class ?class) (sub-type nil))
=>
  (printout error ?goal " of class " ?class " has no sub-type" crlf)
)
