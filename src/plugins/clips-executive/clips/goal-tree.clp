
;---------------------------------------------------------------------------
;  goal-tree.clp - CLIPS executive - goal tree helper functions
;
;  Created: Tue Jun 11 23:07:02 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction goal-tree-update-child (?f ?parent-id ?priority)
	(bind ?sub-goal-id (fact-slot-value ?f id))
	(if (eq ?sub-goal-id nil) then
		(bind ?sub-goal-id (sym-cat (fact-slot-value ?f class) - (gensym*)))
	)
	(modify ?f (id ?sub-goal-id) (parent ?parent-id) (priority ?priority))
)

(deffunction goal-tree-assert-run-one (?class $?fact-addresses)
	(bind ?id (sym-cat RUN-ONE- ?class - (gensym*)))
	(bind ?goal (assert (goal (id ?id) (class ?class) (sub-type RUN-ONE-OF-SUBGOALS))))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-run-all (?class $?fact-addresses)
	(bind ?id (sym-cat RUN-ALL- ?class - (gensym*)))
	(bind ?goal (assert (goal (id ?id) (class ?class) (sub-type RUN-ALL-OF-SUBGOALS))))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-try-all (?class $?fact-addresses)
	(bind ?id (sym-cat TRY-ALL- ?class - (gensym*)))
	(bind ?goal (assert (goal (id ?id) (class ?class) (sub-type TRY-ALL-OF-SUBGOALS))))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-retry (?class ?max-tries ?fact-address)
	(bind ?id (sym-cat RETRY- ?class - (gensym*)))
	(bind ?goal (assert (goal (id ?id) (class ?class) (sub-type RETRY-SUBGOAL)
	                    (params max-tries ?max-tries))))
	(goal-tree-update-child ?fact-address ?id 0)
	(return ?goal)
)

(deffunction goal-tree-assert-timeout (?class ?timeout ?fact-address)
	(bind ?id (sym-cat TIMEOUT- ?class - (gensym*)))
	(bind ?goal (assert (goal (id ?id) (class ?class) (sub-type TIMEOUT-SUBGOAL)
	                          (params timeout ?timeout))))
	(goal-tree-update-child ?fact-address ?id 0)
	(return ?goal)
)
