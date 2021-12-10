;---------------------------------------------------------------------------
;  promises.clp - CLIPS executive - promise representation rules
;
;  Created: Tue Nov 23 2021 00:30:00
;  Copyright  2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate domain-promise
  "A promise is like a domain-fact with the exception that it is not true yet."
  (slot name (type SYMBOL) (default ?NONE))
  (multislot param-values)
  (slot negated (type SYMBOL) (allowed-values TRUE FALSE))
  (slot promising-goal (type SYMBOL))
  (slot promising-agent (type SYMBOL) (default nil))
  (slot active (type SYMBOL) (default FALSE) (allowed-values TRUE FALSE))
  (slot valid-at (type INTEGER))
  (slot do-not-invalidate (type SYMBOL) (default FALSE));indicate if the promise should be retracted with its source goal
)

(deftemplate promise-time
  (slot usecs (type INTEGER))
)

(deffunction sat-or-promised (?sat ?now ?from ?lt)
  (return (or
      (eq ?sat TRUE)
      (and
        (<= ?now ?from)
        (< (- ?from ?now) ?lt)
        (neq ?from -1)
      )
    )
  )
)

(defrule domain-promise-add-promising-agent
  ?dp <- (domain-promise (promising-agent nil))
  (domain-fact (name self) (param-values ?agent))
  =>
  (modify ?dp (promising-agent (sym-cat ?agent)))
)

(defrule domain-promise-activate-promises-on-active-goal
  (goal (id ?goal-id) (mode DISPATCHED))
  ?p <- (domain-promise (promising-goal ?goal-id) (active FALSE) (promising-agent ~nil))
  =>
  (modify ?p (active TRUE))
)

(defrule domain-promise-remove-promises-for-finished-goal
  "If a promise has a goal-id of a goal that doesn't exist, or if the goal is finished,
  evaluated, or retracted, then remove the promise"
  ?d <- (domain-promise (promising-goal ?goal-id) (promising-agent ?agent) (do-not-invalidate FALSE))
  (domain-fact (name self) (param-values ?agent-str&:(eq (sym-cat ?agent-str) ?agent)))
  (or
    (not (goal (id ?goal-id)))
    (goal (id ?goal-id) (mode FINISHED|EVALUATED|RETRACTED))
  )
  =>
  (retract ?d)
)

(defrule domain-promise-remove-promises-for-overtime
  ?d <- (domain-promise (valid-at ?time) (promising-agent ?agent))
  (domain-fact (name self) (param-values ?agent-str&:(eq (sym-cat ?agent-str) ?agent)))
  (promise-time (usecs ?now))
  (test (> ?now ?time))
  =>
  (retract ?d)
)

;------------------------------------- APPLY PROMISE -------------------------------------

;whenever we get a new promise, modify the times
(defrule  promises-update-predicate-positive-promise
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (domain-promise (negated FALSE) (name ?pred) (param-values $?param-values) (active TRUE) (valid-at ?time))

  (pddl-grounding (id ?grounding-id))
  ?base-predicate <- (pddl-predicate (id ?id) (predicate ?pred))
  ?predicate <- (grounded-pddl-predicate (predicate-id ?id)
                                         (grounding ?grounding-id)
                                         (param-values $?param-values)
                                         (promised-from ?p-time))
  (test (or (eq ?p-time -1) (> ?p-time ?time)))
=>
  (modify ?predicate (promised-from ?time))
)

(defrule promises-update-predicate-negative-promise
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (pddl-grounding (id ?grounding-id)
                  (param-names $?grounded-params)
                  (param-values $?grounded-values))

  (domain-promise (negated TRUE) (name ?pred) (param-values $?param-values) (active TRUE) (valid-at ?time))

  ?base-predicate <- (pddl-predicate (id ?id) (predicate ?pred))
  ?predicate <- (grounded-pddl-predicate (predicate-id ?id)
                                         (grounding ?grounding-id)
                                         (param-values $?param-values)
                                         (promised-until ?p-time))
  (test (or (eq ?p-time -1) (> ?p-time ?time)))
=>
  (modify ?predicate (promised-until ?time))
)

(defrule  promises-update-predicate-remove-positive-promise
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (pddl-grounding (id ?grounding-id))
  ?base-predicate <- (pddl-predicate (id ?id) (predicate ?pred))
  ?predicate <- (grounded-pddl-predicate (predicate-id ?id)
                                         (grounding ?grounding-id)
                                         (param-values $?param-values)
                                         (promised-from ?base-time&~-1))

  (not (domain-promise (negated FALSE) (name ?pred) (param-values $?param-values) (active TRUE) (valid-at ?base-time)))
=>
  (modify ?predicate (promised-from -1))
)

(defrule promises-update-predicate-remove-negative-promise
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (pddl-grounding (id ?grounding-id))
  ?base-predicate <- (pddl-predicate (id ?id) (predicate ?pred))
  ?predicate <- (grounded-pddl-predicate (predicate-id ?id)
                                         (grounding ?grounding-id)
                                         (param-values $?param-values)
                                         (promised-until ?base-time&~-1))

  (not (domain-promise (negated TRUE) (name ?pred) (param-values $?param-values) (active TRUE) (valid-at ?base-time)))
=>
  (modify ?predicate (promised-until -1))
)

;---------------------------------------- ATOMIC -----------------------------------------

(defrule promises-update-atomic-formula
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))

  (pddl-formula (id ?parent-base) (type atom))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (promised-from ?base-time-from)
                                    (promised-until ?base-time-until))

  (pddl-predicate (part-of ?parent-base) (id ?child-base))
  (grounded-pddl-predicate (predicate-id ?child-base)
                           (grounding ?grounding-id)
                           (parent-formula ?id)
                           (promised-from ?time-from)
                           (promised-until ?time-until))
  (test
    (or
      (neq ?base-time-from ?time-from)
      (neq ?base-time-until ?time-until)
    )
  )
  =>
  (modify ?parent (promised-from ?time-from) (promised-until ?time-until))
)

;--------------------------------------- NEGATION ----------------------------------------

(defrule promises-update-negated-formula
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type negation))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (promised-from ?base-time-from)
                                    (promised-until ?base-time-until))

  (pddl-formula (part-of ?parent-base) (id ?child-base))
  (grounded-pddl-formula (formula-id ?child-base)
                         (grounding ?grounding-id)
                         (grounded-parent ?id)
                         (promised-from ?time-from)
                         (promised-until ?time-until))

  ;switch the times for negations
  (test
    (or
      (neq ?time-from ?base-time-until)
      (neq ?time-until ?base-time-from)
    )
  )
=>
  (modify ?parent (promised-from ?time-until) (promised-until ?time-from))
)

;--------------------------------- CONJUNCTIVE FORMULAS ----------------------------------

(defrule promises-update-conjunctive-formula-positive
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type conjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (is-satisfied FALSE)
                                    (promised-from ?base-from))

  ;the formula has only satisfied or promised children
  (not
      (and
          (pddl-formula (part-of ?parent-base) (id ?child-base))
          (grounded-pddl-formula (formula-id ?child-base)
                                  (grounding ?grounding-id)
                                  (grounded-parent ?id)
                                  (is-satisfied FALSE)
                                  (promised-from -1))
      )
  )

  ;the max promise time is taken
  (grounded-pddl-formula  (grounding ?grounding-id)
                          (grounded-parent ?id)
                          (is-satisfied FALSE)
                          (promised-from ?from-time&~-1))
  (not
      (grounded-pddl-formula  (grounding ?grounding-id)
                              (grounded-parent ?id)
                              (is-satisfied FALSE)
                              (promised-from ?time&:(> ?time ?from-time)))
  )
  (test (neq ?base-from ?from-time))
  =>
  (modify ?parent (promised-from ?from-time))
)

(defrule promises-update-conjunctive-formula-positive-invalidate-unsat-child
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type conjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (promised-from ~-1))

  ;the formula is not satisfied nor promised
  (pddl-formula (part-of ?parent-base) (id ?child-base))
  (grounded-pddl-formula (formula-id ?child-base)
                          (grounding ?grounding-id)
                          (is-satisfied FALSE)
                          (grounded-parent ?id)
                          (promised-from -1))
  =>
  (modify ?parent (promised-from -1))
)

(defrule promises-update-conjunctive-formula-positive-invalidate-time-mismatch
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type conjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (promised-from ?base-from&~-1))

  ;there is no source for the current promise time
  (not (grounded-pddl-formula (grounding ?grounding-id)
                              (grounded-parent ?id)
                              (promised-from ?base-from))
  )
  =>
  (modify ?parent (promised-from -1))
)

(defrule promises-update-conjunctive-formula-negative
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type conjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (is-satisfied TRUE)
                                    (promised-until ?base-until))

  ;the min promise time is taken
  (grounded-pddl-formula  (grounding ?grounding-id)
                          (grounded-parent ?id)
                          (promised-until ?until-time&~-1))
  (not
      (grounded-pddl-formula  (grounding ?grounding-id)
                              (grounded-parent ?id)
                              (promised-until ?time&:(< ?time ?until-time)))
  )
  (test (neq ?base-until ?until-time))
  =>
  (modify ?parent (promised-until ?until-time))
)

(defrule promises-update-conjunctive-formula-negative-invalidate
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type conjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (promised-until ~-1))

  ;there is a child that is not satisfied and has no promise
  (not
    (and
      (pddl-formula (part-of ?parent-base) (id ?child-base))
      (grounded-pddl-formula (formula-id ?child-base)
                              (grounding ?grounding-id)
                              (is-satisfied TRUE)
                              (grounded-parent ?id)
                              (promised-until ~-1))
    )
  )
  =>
  (modify ?parent (promised-until -1))
)

(defrule promises-update-conjunctive-formula-negative-invalidate-time-mismatch
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type conjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (promised-until ?base-until&~-1))

  (not (grounded-pddl-formula (grounding ?grounding-id)
                              (grounded-parent ?id)
                              (promised-until ?base-until))
  )
  =>
  (modify ?parent (promised-until -1))
)

;--------------------------------- DISJUNCTIVE FORMULAS ----------------------------------

(defrule promises-update-disjunctive-formula-positive
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type disjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (is-satisfied FALSE)
                                    (promised-from ?base-from))

  ;the MIN promise time is taken
  (grounded-pddl-formula  (grounding ?grounding-id)
                          (grounded-parent ?id)
                          (is-satisfied FALSE)
                          (promised-from ?from-time&~-1))
  (not
      (grounded-pddl-formula  (grounding ?grounding-id)
                              (grounded-parent ?id)
                              (is-satisfied FALSE)
                              (promised-from ?time&:(and (neq ?time -1) (< ?time ?from-time))))
  )
  (test (neq ?base-from ?from-time))
  =>
  (modify ?parent (promised-from ?from-time))
)

(defrule promises-update-disjunctive-formula-positive-invalidate
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type disjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (is-satisfied TRUE)
                                    (promised-from ?base-from&~-1))
  =>
  (modify ?parent (promised-from -1))
)

(defrule promises-update-disjunctive-formula-positive-invalidate-time-mismatch
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type disjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (promised-from ?base-from&~-1))

  (not (grounded-pddl-formula (grounding ?grounding-id)
                              (grounded-parent ?id)
                              (promised-from ?base-from))
  )
  =>
  (modify ?parent (promised-from -1))
)

(defrule promises-update-disjunctive-formula-negative
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type disjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (is-satisfied TRUE)
                                    (promised-until ?base-until))

  ;all satisfied children have a negative promise attached
  (not
      (grounded-pddl-formula  (grounding ?grounding-id)
                              (grounded-parent ?id)
                              (is-satisfied TRUE)
                              (promised-until -1))
  )

  ;the max promise time is taken
  (grounded-pddl-formula  (grounding ?grounding-id)
                          (grounded-parent ?id)
                          (is-satisfied TRUE)
                          (promised-until ?until-time&~-1))
  (not
      (grounded-pddl-formula  (grounding ?grounding-id)
                              (grounded-parent ?id)
                              (is-satisfied TRUE)
                              (promised-until ?time&:(and (neq ?time -1) (> ?time ?until-time))))
  )
  (test (neq ?base-until ?until-time))
  =>
  (modify ?parent (promised-until ?until-time))
)

(defrule promises-update-disjunctive-formula-negative-invalidate-unsatisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type disjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (is-satisfied FALSE)
                                    (promised-until ~-1))
  =>
  (modify ?parent (promised-until -1))
)

(defrule promises-update-disjunctive-formula-negative-invalidate-satisfied-child
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type disjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (is-satisfied TRUE)
                                    (promised-until ~-1))
  (grounded-pddl-formula  (grounding ?grounding-id)
                          (grounded-parent ?id)
                          (is-satisfied TRUE)
                          (promised-until -1))
  =>
  (modify ?parent (promised-until -1))
)

(defrule promises-update-disjunctive-formula-negative-invalidate-time-mismatch
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type disjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (grounding ?grounding-id)
                                    (promised-until ?base-until&~-1))

  (not (grounded-pddl-formula (formula-id ?child-base)
                          (grounding ?grounding-id)
                          (grounded-parent ?id)
                          (promised-until ?base-until))
  )
  =>
  (modify ?parent (promised-until -1))
)

;----------------------------------------- DEBUG -----------------------------------------

; (defrule promises-show-promised-for
;   (goal-class (class ?class) (id ?cid) (sub-type ?subtype))
;   (pddl-formula (part-of ?cid) (id ?formula-id))
;   (grounded-pddl-formula (formula-id ?formula-id) (grounding ?grounding-id) (promised-from ?from-time&~-1))
;   (pddl-grounding (id ?grounding-id))
;   (promise-time (usecs ?now))
;   =>
;   (printout t crlf crlf "Goal " ?class " (" ?cid ") is promised for " (- ?from-time ?now) "s at " ?from-time crlf crlf)
; )
; (defrule promises-show-promised-until
;   (goal-class (class ?class) (id ?cid) (sub-type ?subtype))
;   (pddl-formula (part-of ?cid) (id ?formula-id))
;   (grounded-pddl-formula (formula-id ?formula-id) (grounding ?grounding-id) (promised-until ?until-time&~-1))
;   (pddl-grounding (id ?grounding-id))
;   (promise-time (usecs ?now))
;   =>
;   (printout t crlf crlf "Goal " ?class " (" ?cid ") is promised until " (- ?until-time ?now ) "s at " ?until-time crlf crlf)
; )
