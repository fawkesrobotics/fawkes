;---------------------------------------------------------------------------
;  goal-classes.clp - Define the goal classes and their preconditions
;
;  Created: Thu Dec 2 2021
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


; ------------------------- ASSERT GOAL CLASSES -----------------------------------


(defrule goal-class-create-get-container-and-fill
    (domain-constant (type location) (value ?mine))
    (domain-fact (name location-is-mine) (param-values ?mine))

    (not (goal-class (class FILL-CONTAINER) (sub-type SIMPLE) (meta mine ?mine)))
    =>
    (assert
        (goal-class (class FILL-CONTAINER)
                    (id (sym-cat FILL-CONTAINER - ?mine))
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (meta mine ?mine)
                    (param-names     r     base     mine     c)
                    (param-constants nil   BASE     ?mine    nil)
                    (param-types     robot location location container)
                    (param-quantified)
                    (preconditions "
                        (and
                            (robot-at ?r ?base)
                            (robot-can-carry ?r)
                            (container-for-robot ?c ?r)
                            (container-at ?c CONTAINER-DEPOT)
                            (location-is-free ?mine)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-deliver-to-machine
    (domain-constant (type machine) (value ?machine))
    (domain-constant (type location) (value ?side))
    (domain-fact (name location-is-part-of-machine) (param-values ?side ?machine))
    (domain-fact (name location-is-machine-input) (param-values ?side))

    (not (goal-class (class DELIVER) (sub-type SIMPLE) (meta machine ?machine)))
    =>
    (assert
        (goal-class (class DELIVER)
                    (id (sym-cat DELIVER - ?machine))
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (meta machine ?machine)
                    (param-names     r     side     machine  c         material)
                    (param-constants nil   ?side    ?machine nil       nil)
                    (param-types     robot location machine  container material)
                    (param-quantified)
                    (preconditions "
                        (and
                            (robot-carries ?r ?c)
                            (container-filled ?c ?material)
                            (location-is-free ?side)
                            (location-is-machine-input ?side)
                            (location-is-part-of-machine ?side ?machine)
                            (machine-in-state ?machine IDLE)
                            (machine-for-material ?machine ?material)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-start-machine
    (domain-constant (type machine) (value ?machine))
    (not (goal-class (class START-MACHINE) (meta machine ?machine)))
    =>
    (assert
        (goal-class (class START-MACHINE)
                    (id (sym-cat START-MACHINE - ?machine))
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (meta machine ?machine)
                    (param-names     r     side     machine)
                    (param-constants nil   nil      ?machine)
                    (param-types     robot location machine)
                    (param-quantified)
                    (preconditions "
                        (and
                            (location-is-machine-input ?side)
                            (location-is-part-of-machine ?side ?machine)
                            (machine-in-state ?machine FILLED)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-clean-machine
    (domain-constant (type machine) (value ?machine))
    (domain-constant (type location) (value ?side))
    (domain-fact (name location-is-part-of-machine) (param-values ?side ?machine))
    (domain-fact (name location-is-machine-output) (param-values ?side))
    (not (goal-class (class CLEAN-MACHINE) (meta machine ?machine)))
    =>
    (assert
        (goal-class (class CLEAN-MACHINE)
                    (id (sym-cat CLEAN-MACHINE - ?machine))
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (meta machine ?machine)
                    (param-names     r     side     machine  c         mat)
                    (param-constants nil   ?side    ?machine nil       nil)
                    (param-types     robot location machine  container material)
                    (param-quantified)
                    (preconditions "
                        (and
                            (robot-carries ?r ?c)
                            (container-can-be-filled ?c)
                            (location-is-free ?side)
                            (location-is-machine-output ?side)
                            (location-is-part-of-machine ?side ?machine)
                            (machine-in-state ?machine READY)
                            (machine-makes-material ?machine ?mat)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-deliver-xenonite
    (not (goal-class (class DELIVER-XENONITE)))
    =>
    (assert
        (goal-class (class DELIVER-XENONITE)
                    (id DELIVER-XENONITE)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     r     c)
                    (param-constants nil   nil)
                    (param-types     robot container)
                    (param-quantified)
                    (preconditions "
                        (and
                            (robot-carries ?r ?c)
                            (container-filled ?c XENONITE)
                            (location-is-free STORAGE-INPUT)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)
; ------------------------- ASSERT GOALS -----------------------------------

(defrule goal-class-assert-get-container-and-fill
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (class PRODUCTION-RUN-ONE) (id ?parent) (meta $? host ?r))
    (goal-class (class ?class&FILL-CONTAINER) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?base ?mine ?c))

    (not (goal (sub-type ?subtype) (parent ?parent) (mode ~FORMULATED)))
    (not (goal (class ?class) (params robot ~?r $? mine ?mine $?)))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            container ?c
                            start ?base
                            mine ?mine
                    )
                    (required-resources )
    ))

    ;assert promises resulting from the plan-action of this goal
    ; (assert

    ; )
)

(defrule goal-class-assert-deliver-to-machine
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (class PRODUCTION-RUN-ONE) (id ?parent) (meta $? host ?r))
    (goal-class (class ?class&DELIVER) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?side ?machine ?c ?mat))

    (not (goal (sub-type ?subtype) (parent ?parent) (mode ~FORMULATED)))
    (not (goal (class ?class) (params robot ~?r side ?side machine ?machine $?)))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            side ?side
                            machine ?machine
                            container ?c
                            material ?mat
                    )
                    (required-resources )
    ))

    ;assert promises resulting from the plan-action of this goal
    ; (assert

    ; )
)

(defrule goal-class-assert-start-machine
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (class PRODUCTION-RUN-ONE) (id ?parent) (meta $? host ?r))
    (goal-class (class ?class&START-MACHINE) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?side ?machine))

    (not (goal (sub-type ?subtype) (parent ?parent) (mode ~FORMULATED)))
    (not (goal (class ?class) (params robot ~?r side ?side machine ?machine)))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            side ?side
                            machine ?machine
                    )
                    (required-resources )
    ))

    ;assert promises resulting from the plan-action of this goal
    ; (assert

    ; )
)

(defrule goal-class-assert-clean-machine
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (class PRODUCTION-RUN-ONE) (id ?parent) (meta $? host ?r))
    (goal-class (class ?class&CLEAN-MACHINE) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?side ?machine ?c ?mat))

    (not (goal (sub-type ?subtype) (parent ?parent) (mode ~FORMULATED)))
    (not (goal (class ?class) (params robot ~?r side ?side machine ?machine $?)))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            side ?side
                            machine ?machine
                            container ?c
                            material ?mat
                    )
                    (required-resources )
    ))

    ;assert promises resulting from the plan-action of this goal
    ; (assert

    ; )
)

(defrule goal-class-assert-deliver-xenonite
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (class PRODUCTION-RUN-ONE) (id ?parent) (meta $? host ?r))
    (goal-class (class ?class&DELIVER-XENONITE) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?c))

    (not (goal (sub-type ?subtype) (parent ?parent) (mode ~FORMULATED)))
    (not (goal (class ?class) (params robot ~?r $?)))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            container ?c
                    )
                    (required-resources )
    ))

    ;assert promises resulting from the plan-action of this goal
    ; (assert

    ; )
)