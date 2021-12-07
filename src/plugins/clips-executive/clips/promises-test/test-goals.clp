;---------------------------------------------------------------------------
;  test-goals.clp - Unit-test like goals that can be used for testing promises
;
;  Created: Tue Dec 7 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

(defrule goal-class-create-test-goal-conjunction
    (not (goal-class (class TEST-GOAL-CONJUNCTION)))
    =>
    (assert
        (goal-class (class TEST-GOAL-CONJUNCTION)
                    (id TEST-GOAL-CONJUNCTION)
                    (param-quantified)
                    (preconditions "
                        (and
                            (machine-in-state MACHINE1 READY)
                            (machine-in-state MACHINE1 IDLE)
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-assert-test-goal-conjunction
    (goal-class (class TEST-GOAL-CONJUNCTION) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied FALSE) (promised-from ?from&~-1) (promised-until ?until) (grounding ?grounding-id))
    (domain-fact (name machine-in-state) (param-values MACHINE1 READY))
    =>
    (printout t "TESTGOAL: Conjunction " ?from " " ?until crlf)
)

(defrule goal-class-create-test-goal-negation
    (not (goal-class (class TEST-GOAL-NEGATION)))
    =>
    (assert
        (goal-class (class TEST-GOAL-NEGATION)
                    (id TEST-GOAL-NEGATION)
                    (param-quantified)
                    (preconditions "
                        (and
                            (not (machine-in-state MACHINE1 IDLE))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-assert-test-goal-negation
    (goal-class (class TEST-GOAL-NEGATION) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied FALSE) (promised-from ?from&~-1) (promised-until ?until) (grounding ?grounding-id))
    =>
    (printout t "TESTGOAL: Negation " ?from " " ?until crlf)
)

(defrule goal-class-create-test-goal-disjunction
    (not (goal-class (class TEST-GOAL-DISJUNCTION)))
    =>
    (assert
        (goal-class (class TEST-GOAL-DISJUNCTION)
                    (id TEST-GOAL-DISJUNCTION)
                    (param-quantified)
                    (preconditions "
                        (and
                            (or
                                (machine-in-state MACHINE1 READY)
                                (machine-in-state MACHINE1 FILLED)
                            )
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-assert-test-goal-disjunction
    (goal-class (class TEST-GOAL-DISJUNCTION) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied FALSE) (promised-from ?from&~-1) (promised-until ?until) (grounding ?grounding-id))
    =>
    (printout t "TESTGOAL: Disjunction " ?from " " ?until crlf)
)
