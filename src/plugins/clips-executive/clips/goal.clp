;---------------------------------------------------------------------------
;  goal.clp - CLIPS executive - goal representation
;
;  Created: Fri Jun 01 16:40:41 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
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
     intended to achieve the desired goal, or the respective sub-goals
     have been formulated.
   - COMMITTED
     The goal and (one of) its plan(s) is intended to be executed to achieve
     the respective goal. Committing may require to obtain specific resources
     indicated by the plan.
   - DISPATCHED
     An execution mechanism has started to execute the plan associated with
     the goal, for example issuing action commands.
   - FINISHED
     The goal has finished, meaning that no further actions of plans
     belonging to this goal will be executed. The outcome field indicates
     whether the goal was completed, rejected, or has failed.
   - EVALUATED
     After a goal has finished, the goal reasoner evaluates the impact of
     the (succeeded, rejected, or failed) goal on its own belief.
     This deviates from the goal lifecycle by Roberts et al in that it is
     not only relevant during failure, but always after finishing a goal.
   - RETRACTED
     After the goal has been evaluated, some cleanup (e.g., releasing acquired
     resources) may be necessary before the goal can be destroyed. Once all
     cleanup has been completed, the goal can be retracted. A retracted goal
     is being cleaned up and scheduled for destruction, no further operations
     are allowed on the goal.

   Once a goal reached the FINISHED mode the outcome determines whether it
   was successful or failed:
   - COMPLETED
     The goal been achieved and plan execution finished successfully.
   - FAILED
     The goal has failed and can no longer be pursued unless being reset.
   - REJECTED
     The goal reasoner deemed this goal not to be feasible or desirable
     (any more). A goal may only be rejected from the FORMULATED, SELECTED,
     or EXPANDED mode. Once we have COMMITTED to a goal it can only be stopped
     by completion or failure.
   - UNKNOWN
     The goal has not reached its FINISHED mode, yet.

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
	(slot sub-type (type SYMBOL))
  (slot parent (type SYMBOL))
	(slot mode (type SYMBOL)
    (allowed-values FORMULATED SELECTED EXPANDED COMMITTED DISPATCHED FINISHED
                    EVALUATED RETRACTED))
	(slot outcome (type SYMBOL)
    (allowed-values UNKNOWN COMPLETED FAILED REJECTED))
	(slot message (type STRING))
	; higher number entails higher priority
	; A goal might be preferred for selection, expansion, or execution depending
	; on its priority. Some spec chains may support this, others not.
	(slot priority (type INTEGER) (default 0))
	; Parameters that a goal reasoner or expander might need to evaluate goal.
	; It is recommended to use a rich descriptive structure similar to wm-fact
	; keys. Examples:
	; - (params order O1 robot R1)
	; - (params machine RS1 robots [ R1 R2 ])
	; - (params text "Remember the Cant")
	; - (params location { x 0.0 y 1.0 })
	(multislot params)

	; Goal meta data allows the goal reasoner to store arbitrary
	; information that is necessary to handle the proper operation of
	; the goal and that is not passed as a parameter. For example, it
	; could contain the number of tries performed for a goal.
	(multislot meta)

  (multislot required-resources)
  (multislot acquired-resources)

	; Once committing to a goal, this identifies the goal or plan to
	; which we have committed, in particular if there are multiple
	; possible sub-goals or associated plans.
	(slot committed-to (type SYMBOL))
)

(deffunction goal-retract-goal-tree (?id)
	"Recursively retract a goal tree rooted at the goal with the given ID."
	(do-for-fact ((?g goal)) (eq ?g:id ?id)
		(delayed-do-for-all-facts ((?sub-goal goal)) (eq ?sub-goal:parent ?g:id)
			(goal-retract-goal-tree ?sub-goal:id)
		)
		(plan-retract-all-for-goal ?id)
		(retract ?g)
	)
)

(defrule goal-retract
	(confval (path "/clips-executive/automatic-goal-retraction") (type BOOL) (value TRUE))
	?g <- (goal (id ?id) (mode RETRACTED) (parent ?parent) (acquired-resources))
	; we can retract if there is no parent goal, or the parent goal is a
	; MAINTAIN goal, i.e., it is never RETRACTED itself.
	(or (goal (id ?id) (parent nil))
			(goal (id ?parent) (type MAINTAIN)))
	=>
	(printout t "Retracting goal " ?id crlf)
	(goal-retract-goal-tree ?id)
)

(defrule goal-cleanup-plans
	?g <- (goal (id ?id) (mode RETRACTED))
	(exists (plan (goal-id ?id)))
	=>
	(plan-retract-all-for-goal ?id)
)
