;---------------------------------------------------------------------------
;  goal-class.clp - CLIPS executive - goal class representation
;
;  Created: Sun Sep 12 20:44:00 2021
;  Copyright  2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate goal-class
    "
    A description of the properties of a goal class specifying its
    preconditions, effects, parameters and types.
    In the goal lifecycle, goal classes are used to specify the preconditions
    for formulation of goals in a general way. These preconditions are grounded
    automatically similarly to plan actions and subsequently checked for
    satisfcation.
    "
    (slot class (type SYMBOL))
    (slot id (type SYMBOL))
    (slot type (type SYMBOL) (allowed-values ACHIEVE MAINTAIN) (default ACHIEVE))
    (slot sub-type (type SYMBOL))
    (multislot meta)
    (multislot param-names)
    (multislot param-constants)
    (multislot param-quantified)
    (multislot param-types)
    (slot lookahead-time (type INTEGER) (default 0))
    (slot preconditions (type STRING))
    (slot effects (type STRING))
)

(deffunction goal-class-create-grounding
    "Go through the list of parameters and build up a grounding. For values that have an
    entry that is not nil in param-constants, assign that value. For values that are quantified
    leave them empty, as they will be quantified in the formula. For values that are neither
    quantified nor constnats, assign the value based on the entry in the type field by
    creating one grounding for each possible value or constant of that type in the current
    world. "
    (?goal-class-id ?param-types ?param-names ?param-names-left ?param-constants ?param-quantified ?param-values)

    (if (> (length$ ?param-types) 0)
        then
        (if (neq (nth$ 1 ?param-constants) nil)
            then
                (bind ?param-values-new (insert$ ?param-values (+ 1 (length$ ?param-values)) (nth$ 1 ?param-constants)))
                (goal-class-create-grounding ?goal-class-id
                                             (delete$ ?param-types 1 1)
                                             (delete$ ?param-names 1 1)
                                             ?param-names-left
                                             (delete$ ?param-constants 1 1)
                                             ?param-quantified
                                             ?param-values-new)
            else
            (if (not (member$ (nth$ 1 ?param-names) ?param-quantified))
                then
                    ;ground by domain objects
                    (do-for-all-facts ((?object domain-object)) (eq ?object:type (nth$ 1 ?param-types))
                        (bind ?param-values-new (insert$ ?param-values (+ 1 (length$ ?param-values)) ?object:name))
                        (goal-class-create-grounding ?goal-class-id
                                                     (delete$ ?param-types 1 1)
                                                     (delete$ ?param-names 1 1)
                                                     ?param-names-left
                                                     (delete$ ?param-constants 1 1)
                                                     ?param-quantified
                                                      ?param-values-new)
                    )
                    ;ground by domain constants
                    (do-for-all-facts ((?constant domain-constant)) (eq ?constant:type (nth$ 1 ?param-types))
                        (bind ?param-values-new (insert$ ?param-values (+ 1 (length$ ?param-values)) ?constant:value))
                        (goal-class-create-grounding ?goal-class-id
                                                     (delete$ ?param-types 1 1)
                                                     (delete$ ?param-names 1 1)
                                                     ?param-names-left
                                                     (delete$ ?param-constants 1 1)
                                                     ?param-quantified
                                                     ?param-values-new)
                    )
                else
                    (bind ?param-values-new (insert$ ?param-values (+ 1 (length$ ?param-values)) nil))
                    (goal-class-create-grounding ?goal-class-id
                                                 (delete$ ?param-types 1 1)
                                                 (delete$ ?param-names 1 1)
                                                 ?param-names-left
                                                 (delete$ ?param-constants 1 1)
                                                 ?param-quantified
                                                 ?param-values-new)
            )
        )
        else
        (if (not (any-factp ((?grounding pddl-grounding)) (and (eq ?grounding:formula-root ?goal-class-id)
                                                               (eq ?grounding:param-values ?param-values))))
            then
                (printout t "Adding new Groundings: " ?param-values " for " ?goal-class-id crlf)
                (bind ?grounding-id (sym-cat "grounding-" ?goal-class-id "-" (gensym*)))
                (assert (pddl-grounding (param-names ?param-names-left)
                                        (param-values ?param-values)
                                        (formula-root ?goal-class-id)
                                        (id ?grounding-id)
                        )
                )
        )
    )
)

(defrule goal-class-grounding
    "If there is a domain-object matching the type of a parameter of a goal class, or if
    no grounding exists for the goal class at all (e.g. for fully quantified formulas)
    generate the possible groundings that do not exist yet. "
    (domain-facts-loaded)
    (goal-class (id ?class-id)
                (class ?class)
                (param-names $?param-names)
                (param-constants $?param-constants)
                (param-types $?param-types)
                (param-quantified $?param-quantified)
    )

    (pddl-formula (part-of ?class-id) (id ?formula-id))

    (or
        (and
            (domain-object (name ?object) (type ?type&:
                        (and
                            (member$ ?type ?param-types)
                            (eq (nth$ (member$ ?type ?param-types) ?param-constants) nil)
                        ))
            )

            (not
                (and
                    (pddl-grounding (id ?grounding-id)
                                    (formula-root ?formula-id)
                                    (param-names $?param-names)
                                    (param-values $? ?object $?))
                )
            )
        )
        (not
            (and
                (pddl-grounding (id ?grounding-id) (formula-root ?formula-id))
            )
        )
    )
    =>
    (goal-class-create-grounding ?class-id
                                 ?param-types
                                 ?param-names
                                 ?param-names
                                 ?param-constants
                                 ?param-quantified
                                 (create$ ))
)

(defrule goal-class-retract-unbased-grounding
    "If there is a grounding of a goal class using a value that does not exist anymore,
    retract the grounding to trigger the removal of the formula. "
    (goal-class (id ?class-id)
                (class ?class)
                (param-names $?param-names)
                (param-constants $?param-constants)
                (param-types $?param-types)
                (param-quantified $?param-quantified)
    )
    (pddl-formula (part-of ?class-id) (id ?formula-id))
    ?g <- (pddl-grounding (id ?grounding-id) (param-values $? ?value&~nil $?) (formula-root ?formula-id))
    (not (and  (domain-object (name ?value))
               (domain-constant (value ?value))
        )
    )
    =>
    (printout t "Retracting grounding" ?grounding-id " because value " ?value " is " crlf)
    (retract ?g)
)

(defrule goal-class-assert-precondition-formula-for-class
    "If there is a goal class that doesn't have its precondition formula
    translated to a set of PDDL formula facts yet, parse its formula string."
    (goal-class (class ?class) (id ?cid) (preconditions ?prec))
    (not (pddl-formula (part-of ?class)))
    =>
    (parse-pddl-formula ?prec (str-cat ?cid))
)
