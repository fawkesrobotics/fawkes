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
   executive. The class allows users to implement the specific handling for
   specific kinds of goals of that very class, and thus to match these goals
   in goal reasoning rules' antecendents.

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

	; The sub-type of a goal describes the inherent behavior of a
	; goal. It is used to specify well-known goals with specific
	; semantics, for example to run all sub-goals or retry a given
	; sub-goal. The default sub-type is nil and means an entirely
	; user-specificed goal, i.e., the user takes control of all stages
	; in the goal life-cycle.
	(slot sub-type (type SYMBOL))

	; To form goal trees, a parent goal can be specified to which this
	; goal belongs. Unless a well-known sub-goal sub-type (e.g.,
	; RUN-ALL-OF-SUBGOALS, TRY-ALL-OF-SUBGOALS, or RETRY-SUBGOAL) is
	; used for the parent goal, the user must take take care of all
	; stages of the lifecycle.
	(slot parent (type SYMBOL))

	; The mode describes the stage in the goal lifecycle the goal is in, see
	; general template documentation above.
	(slot mode (type SYMBOL)
	           (allowed-values FORMULATED SELECTED EXPANDED COMMITTED
	   	                       DISPATCHED FINISHED EVALUATED RETRACTED))
	; The outcome of a goal is initially UNKNOWN, and at the latest when
	; entering the FINISHED mode must be determined to be either
	; successfully COMPLETED, or FAILED. A goal may be REJECTED before
	; it is in COMMITTED mode. The general distinction is that goals can
	; be REJECTED while they have not started execution, i.e., before
	; they are enter the COMITTED mode, and may only complete or fail
	; afterwards.
	(slot outcome (type SYMBOL)
	              (allowed-values UNKNOWN COMPLETED FAILED REJECTED))

	; In case of a disturbance, give machine-readable information.
	; This field may contain a number of disturbances and are meant to
	; inform, e.g., the goal reasoner or some execution monitoring entity,
	; possibly to take corrective action or even abort a goal.
	; Warnings MUST always start with a symbol identifying the warning,
	; e.g., ACTION-OVERTIME, and may be followed by an arbitrary number
	; of additional values specifying the warning closer, e.g., to
	; indicate that a move operation took too long, the warning could
	; be (ACTION-OVERTIME 20 move), to indicate that the action with
	; the ID 20 in the committed plan using the operator `move` took
	; too long. A warning may appear more than once, e.g., to indicate
	; multiple overtime situations in the executed plan.
	; The intention is to easily allow for the occurrence of any
	; warning of a specified type, but also to acquire more information
	; by components which are able to handle a *specific* warning.
	; This means that we do not strive for a generic warning processing
	; (other than "there is any warning") and therefore we do not introduce
	; a separator in-between warnings.
	(multislot warning)

	; In case of an error, give machine-readable information about the
	; error. The first entry in the field MUST be a symbol identifying
	; the general reason of the error, e.g., SKILL-FAILED. The intention
	; of the field is to allow for error handling and recovery.
	(multislot error)
	; A human-readable message describing an error. This message is
	; passed on to frontends and displayed to the user.
 	(slot message (type STRING))

	; higher number entails higher priority
	; A goal might be preferred for selection, expansion, or execution depending
	; on its priority. Some spec chains may support this, others not.
	(slot priority (type FLOAT) (default 0.0))
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
	; Goal meta data may depend on the application, hence application-specific
	; templates may be used to attach information.
	; The only requirement for custom templates is to contain a slot 'goal-id'
	; which is used to keep track of fact updates.
	; When formulating a goal it is sufficient to either set the template name or
	; a fact-index of the meta fact.
	; If 'meta-template' is set:
	;   - if no meta fact of that template is present matching the goal id,
	;     then a new meta fact is asserted with default values
	;   - else the existing meta fact with the matching goal id is attached to
	;     the goal
	; If 'meta-fact' is set the meta-template slot is updated automatically.
	; Also, if the goal-id field does not match the goals id, it is updated.
	(slot meta-fact (type INTEGER) (default 0))
	(slot meta-template (type SYMBOL))

	; Resource handling for the given goal.  A goal may list a number of
	; required resources. The resources can be of different types and
	; specific handlers need to take care of acquiring these
	; resources. Acquisition generally happens once committed to a
	; goal. A goal may only enter the DISPATCHED mode if the required
	; resources are a subset of the acquired resources. Once a goal has
	; been RETRACTED the resources must be deallocated again by the
	; handlers. Only when the acquired resources are empty is a goal
	; completed removed. For sub-goal types, parent goals shall not
	; advance beyond a goal until all resources for a goal have been
	; deallocated. The automatic cleanup waits for all resources to be
	; deallocated.
	(multislot required-resources (type SYMBOL))
	(multislot acquired-resources (type SYMBOL))

	; Once committing to a goal, this identifies the goal or plan to
	; which we have committed, in particular if there are multiple
	; possible sub-goals or associated plans.
	(slot committed-to (type SYMBOL))

	; Amount of information to print. If set to quiet, regular events such as a
	; committed or dispatched goal will only be logged to the debug log.
	(slot verbosity (type SYMBOL) (allowed-values QUIET DEFAULT NOISY) (default DEFAULT))

	; Indicates whether the given goal is executable, meaning if it is generally
	; feasible to expand it in the current situation.
	(slot is-executable (type SYMBOL) (allowed-values TRUE FALSE)
	                    (default FALSE))
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

(defrule goal-print-error
  (goal (id ?id) (mode RETRACTED) (outcome FAILED)
        (class ?class) (error $?error&:(> (length$ ?error) 0)) (message ?msg&~""))
 =>
  (printout error "Goal " ?id " (" ?class ") failed: " ?msg " " ?error crlf)
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
	(confval (path "/clips-executive/automatic-goal-retraction") (type BOOL) (value TRUE))
	?g <- (goal (id ?id) (mode RETRACTED))
	(exists (plan (goal-id ?id)))
	=>
	(plan-retract-all-for-goal ?id)
)

(defrule goal-meta-fact-template-missing-goal-id
	(declare (salience ?*SALIENCE-FIRST*))
	(goal (id ?id) (meta-fact ~0) (meta-template ?t&~nil))
	(test (not (deftemplate-slot-existp ?t goal-id)))
	=>
	(printout error "Goal " ?id " uses meta fact '" ?t
	                "' without slot 'goal-id'" crlf)
)

(defrule goal-meta-ambiguous-meta-facts
	(declare (salience ?*SALIENCE-HIGH*))
	?g <- (goal (id ?id) (meta-fact ~0) (meta-template ?t&~nil))
	(test (> (length$ (find-all-facts ((?fact ?t)) (eq ?fact:goal-id ?id))) 1))
	=>
	(printout error "Goal " ?id " has ambiguous meta facts" crlf)
)

(defrule goal-update-meta-template
" A goal meta is set via fact-index. Update the template name accordingly."
	(declare (salience ?*SALIENCE-HIGH*))
	?g <- (goal (id ?id) (meta-fact ?f&:(and (> ?f 0) (fact-existp ?f)))
	            (meta-template ?t&:(neq ?t (fact-relation ?f))))
	=>
	(modify ?g (meta-template (fact-relation ?f)))
)

(defrule goal-meta-fact-init-fact-id
" The meta fact template is specified and a goal meta fact exists for the goal.
  Update the goal meta fact-index.
"
	(declare (salience ?*SALIENCE-HIGH*))
	?g <- (goal (id ?id) (meta-fact 0) (meta-template ?t&~nil))
	(test (any-factp ((?f ?t)) (eq ?f:goal-id ?id)))
	=>
	(do-for-fact ((?f ?t)) (eq ?f:goal-id ?id)
	  (modify ?g (meta-fact (fact-index ?f)))
	)
)

(defrule goal-meta-fact-init-goal-id
" The meta-fact is already set, but the goal id does not match, update the
  goal-id of the goal meta fact.
"
	(declare (salience ?*SALIENCE-HIGH*))
	?g <- (goal (id ?id) (meta-fact ?f&~0) (meta-template ?t&~nil))
	(test (and (fact-existp ?f) (neq (fact-slot-value ?f goal-id) ?id)))
	=>
	(modify ?f (goal-id ?id))
)

(defrule goal-meta-fact-update
" The stored meta fact does not exist anymore, it was updated or deleted.
  If another goal meta fact exists with matching goal id, then update the
  stored fact-index, else remove the outdated goal-meta information.
"
	(declare (salience ?*SALIENCE-HIGH*))
	?g <- (goal (id ?id) (meta-fact ?f&~0) (meta-template ?t&~nil))
	(time $?) ; Re-evaluate the LHS continuously
	(test (not (fact-existp ?f)))
	=>
	(if (not (do-for-fact ((?new-fact ?t)) (eq ?new-fact:goal-id ?id)
	                      (modify ?g (meta-fact (fact-index ?new-fact)))))
	 then
		(modify ?g (meta-fact 0) (meta-template nil))
	)
)

(defrule goal-cleanup-meta-facts
	(confval (path "/clips-executive/automatic-goal-retraction") (type BOOL) (value TRUE))
	(goal (meta-fact ?f&~0) (mode RETRACTED))
	(test (fact-existp ?f))
	=>
	(retract ?f)
)

(defrule goal-assert-default-meta
" A goal meta template is specified but no corresponding fact exits,
  assert default goal meta fact."
	(declare (salience ?*SALIENCE-HIGH*))
	?g <- (goal (id ?id) (meta-fact 0) (verbosity ?v)
	            (meta-template ?t&:(member$ ?t (get-deftemplate-list)))
	            (mode FORMULATED))
	=>
	(if (not (any-factp ((?fact ?t)) (eq ?fact:goal-id ?id))) then
		(modify ?g (meta-fact (fact-index (assert-string (str-cat "(" ?t
		        " (goal-id " ?id"))")))))
		(if (eq ?v NOISY) then
			(printout t "Goal " ?id " attached default meta fact from template " ?t crlf)
		)
	)
)

(defrule goal-meta-template-invalid
	(declare (salience ?*SALIENCE-FIRST*))
	?g <- (goal (id ?id) (meta-template ?t&:
	            (and (neq ?t nil)
	                 (not (member$ ?t (get-deftemplate-list)))))
	            (mode ~RETRACTED))
	=>
	(printout error "Goal " ?id " attached goal meta template '"
	                ?t "' unknown" crlf)
)
