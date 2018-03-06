
;---------------------------------------------------------------------------
;  plan.clp - CLIPS executive - plan representation
;
;  Created: Wed Sep 20 15:47:23 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate goal
	"Goal specification.
   A goal is the top-most entity to structure the flow of execution. It
   follows a goal lifecycle indicated by the mode of the goal.

   There are two types of goals:
   ACHIEVE:  a certain target condition is to be achieved through the
             generation and execution of a (possibly ordered) set of actions.
   MAINTAIN: a certain condition is to be maintained during the existence
             of the goal. This may be used for monitoring (the goal fails if
             the condition no longer holds) or for active maintenance
             (sub-goals may be generated to re-achieve the condition).

   The goal lifecycle roughly has the following stages:
   - FORMULATED
     The goal has just come into existence and is to be considered.
   - SELECTED
     The goal has been selected for expansion, that is, a set of actions
     (possibly a sequence or a temporal network) is to be generated
     (for example using a planner, or through a static plan) that
     intends to achieve the given goal.
   - EXPANDED
     The goal has been expanded, that is, there exists at least one plan
     intended to achieve the desired goal.
   - COMMITTED
     The goal and (one of) its plan(s) is intended to be executed to achieve
     the respective goal. Committing may require to obtain specific resources
     indicated by the plan.
   - DISPATCHED
     An execution mechanism has started to execute the plan associated with
     the goal, for example issuing action commands.
   - COMPLETED
     The goal has been achieved and plan execution finished successfully.
   - FAILED
     The goal has failed and can no longer be pursued unless being reset.
   - REJECTED
     The goal reasoner deemed this goal not to be feasible or desirable
     (any more). A goal may only be rejected from the FORMULATED, SELECTED,
     or EXPANDED mode. Once we have COMMITTED to a goal it can only be stopped
     by completion or failure.

   A goal has a unique ID (which must be globally unique among all goals). It
   may be associated to a specific class of goals. That is, there may be
   multiple instances of a certain class of goals, but these must have
   different IDs. A goal ID may appear only once during the lifetime of the
   executive.

   A goal may have a parent goal. This allows for the creation of goal
   hierarchies and sub-goals can contribute to the completion or failure
   of the parent goal. A goal shall fail if any of its sub-goals fails.
   If an ACHIEVE goal has any sub-goals, it shall be COMPLETED once all of
   its sub-goals have been COMPLETED. MAINTAIN goals shall fail if a sub-goal
   fails, but stay COMMITTED or DISPATCHED if a sub-goal is COMPLETED.

   Goals can contain an informative message. This message is intended to
   explain the goal mode to the user. For example, when a goal is REJECTED
   it should state the reason for rejection.
  "
	(slot id (type SYMBOL))
	(slot class (type SYMBOL))
  (slot type (type SYMBOL) (allowed-values ACHIEVE MAINTAIN) (default ACHIEVE))
	(slot mode (type SYMBOL) (allowed-values FORMULATED SELECTED EXPANDED
																					 COMMITTED DISPATCHED COMPLETED FAILED REJECTED))
  (slot parent (type SYMBOL))
	(slot message (type STRING))
)

(deftemplate plan
	(slot id (type SYMBOL))
	(slot goal-id (type SYMBOL))
	(slot cost (type FLOAT))
)

(deftemplate plan-action
	(slot id (type INTEGER))
	(slot plan-id (type SYMBOL))
	(slot action-name (type SYMBOL))
	(multislot param-names)
	(multislot param-values)
	(slot duration (type FLOAT))
	(slot dispatch-time (type FLOAT))
	(slot status (type SYMBOL) (allowed-values FORMULATED PENDING WAITING RUNNING EXECUTION-SUCCEEDED SENSED-EFFECTS-WAIT SENSED-EFFECTS-HOLD EFFECTS-APPLIED FINAL EXECUTION-FAILED FAILED))
  (slot executable (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deffunction plan-action-arg (?param-name ?param-names ?param-values $?default)
	(foreach ?p ?param-names
		(if (eq ?param-name ?p) then (return (nth$ ?p-index ?param-values)))
	)
	(if (> (length$ ?default) 0) then (return (nth$ 1 ?default)))
	(return FALSE)
)

; alternative
; (deftemplate plan-action-parameter
; 	(slot plan-action-id (type INTEGER))
; 	(slot key)
; 	(slot value)
; )
