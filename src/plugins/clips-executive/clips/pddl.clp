;---------------------------------------------------------------------------
;  pddl.clp - Interface to a PDDL planner using robot memory
;
;  Created: Tue 07 Nov 2017 18:36:08 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(deftemplate pddl-plan
  ; The ID of the goal.
  (slot goal-id (type SYMBOL))
  ; The ID of PDDL generation message.
  (slot gen-id (type INTEGER))
  ; The ID of the PDDL Plan message.
  (slot plan-id (type INTEGER))
  ; The current status of this plan.
  (slot status (type SYMBOL)
    (allowed-values
      GEN-PENDING GEN-RUNNING GENERATED PENDING RUNNING PLANNED PLAN-FETCHED
    )
  )
  ; The PDDL formula to plan for.
  (slot goal (type STRING))
)

(deffunction pddl-call (?goal-id ?goal)
  "Call the PDDL planner for the given goal-id with the goal given as string."
  (bind ?m
    (blackboard-create-msg "PddlGenInterface::pddl-gen" "GenerateMessage")
  )
  (blackboard-set-msg-field ?m "goal" ?goal)
  (printout info "Calling PDDL planner for goal '" ?goal "'" crlf)
  (bind ?gen-id (blackboard-send-msg ?m))
  (assert (pddl-plan
    (goal-id ?goal-id) (goal ?goal) (status GEN-PENDING) (gen-id ?gen-id))
  )
)

(defrule pddl-check-if-generation-running
  "Check whether the PDDL generator started."
  ?p <- (pddl-plan (status GEN-PENDING) (gen-id ?gen-id))
  (PddlGenInterface (id "pddl-gen") (msg_id ?gen-id))
  =>
  (printout t "PDDL problem generation started for ID " ?gen-id crlf)
  (modify ?p (status GEN-RUNNING))
)

(defrule pddl-check-if-generation-finished
  "Check whether the PDDL generator finished."
  ?p <- (pddl-plan (status GEN-RUNNING) (gen-id ?gen-id))
  (PddlGenInterface (id "pddl-gen") (msg_id ?gen-id) (final TRUE))
  =>
  (printout t "PDDL problem generation finished for ID " ?gen-id crlf)
  (modify ?p (status GENERATED))
)

(defrule pddl-start-planner
  "Start the actual planner after generating the PDDL problem."
  ?p <- (pddl-plan (status GENERATED) (gen-id ?gen-id))
  =>
  (printout t "Starting to plan " ?gen-id crlf)
  (bind ?m
    (blackboard-create-msg "PddlPlannerInterface::pddl-planner" "PlanMessage")
  )
  (bind ?plan-id (blackboard-send-msg ?m))
  (modify ?p (plan-id ?plan-id) (status PENDING))
)

(defrule pddl-check-if-planner-running
  "Check whether the planner started to plan."
  ?p <- (pddl-plan (status PENDING) (plan-id ?plan-id))
  (PddlPlannerInterface (id "pddl-planner") (msg_id ?plan-id))
  =>
  (modify ?p (status RUNNING))
)

(defrule pddl-check-if-planner-finished
  "Check whether the planner finished planning."
  ?p <- (pddl-plan (status RUNNING) (plan-id ?plan-id))
  (PddlPlannerInterface (id "pddl-planner") (msg_id ?plan-id) (final TRUE)
    (success TRUE))
  =>
  (modify ?p (status PLANNED))
)

(defrule pddl-check-if-planner-failed
  "Check whether the planner finished but has not found a plan."
  ?g <- (goal (id ?goal-id))
  ?p <- (pddl-plan (status RUNNING) (goal-id ?goal-id) (plan-id ?plan-id))
  (PddlPlannerInterface (id "pddl-planner") (msg_id ?plan-id) (final TRUE)
    (success FALSE))
  =>
  (printout error "Planning failed for goal " ?goal-id crlf)
  (modify ?g (mode FINISHED) (outcome FAILED) )
  (retract ?p)
)

(deffunction pddl-get-max-action-id ()
  "Get the max ID of all current action"
  (bind ?i 0)
  (do-for-all-facts ((?a plan-action)) (> ?a:id ?i) (bind ?i ?a:id))
  (return ?i)
)

(defrule pddl-expand-goal
  "Fetch the resulting plan from robot memory and expand the goal."
  ?g <- (goal (id ?goal-id) (mode SELECTED))
  ?t <- (robmem-trigger (name "new-plan") (ptr ?obj))
  ?p <- (pddl-plan
          (status PLANNED)
          (goal-id ?goal-id)
          (plan-id ?plan-id&
            :(eq ?plan-id (bson-get (bson-get ?obj "o") "msg_id")))
        )
  =>
  (printout t "Fetched a new plan!" crlf)
  (progn$ (?action (bson-get-array (bson-get ?obj "o") "actions"))
    (bind ?action-name (sym-cat (bson-get ?action "name")))
    ; FF sometimes returns the pseudo-action REACH-GOAL. Filter it out.
    (if (neq ?action-name REACH-GOAL) then
      (bind ?param-values (bson-get-array ?action "args"))
      ; Convert all parameters to upper-case symbols
      (progn$ (?param ?param-values)
        (bind ?param-values
              (replace$
                ?param-values
                ?param-index ?param-index
                (sym-cat (upcase ?param))
              )
        )
      )
      (assert
        (plan (id ?plan-id) (goal-id ?goal-id))
        (plan-action
          (id ?action-index)
          (goal-id ?goal-id)
          (plan-id ?plan-id)
          (action-name ?action-name)
          (param-values ?param-values)
        )
      )
    )
  )
  (modify ?g (mode EXPANDED))
  (retract ?p)
)

(defrule pddl-clean-up-stale-plans
  "Clean up pddl-plan facts that were left behind.
   This may happen if a goal get unselected while the planner is running."
  ?p <- (pddl-plan (goal-id ?goal-id))
  (not (goal (id ?goal-id) (mode SELECTED)))
  =>
  (retract ?p)
)
