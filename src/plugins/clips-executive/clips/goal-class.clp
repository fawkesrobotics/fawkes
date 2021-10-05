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
    (slot type (type SYMBOL) (allowed-values ACHIEVE MAINTAIN) (default ACHIEVE))
    (slot sub-type (type SYMBOL))
    (multislot param-names)
    (multislot param-constants)
    (multislot param-quantified)
    (multislot param-types)
    (slot preconditions (type STRING))
    (slot effects (type STRING))
)

