;---------------------------------------------------------------------------
;  domain.clp - Representation of a planning domain
;
;  Created: Fri 22 Sep 2017 11:35:49 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate domain-object-type
  "A type in the domain. The type obj must be super-type of all types."
  (slot name (type SYMBOL))
  (slot super-type (type SYMBOL) (default object))
)

(deftemplate domain-object
  "An object in the domain with the given name and type. The type must refer to
   the name of an existing type."
  (slot name)
  (slot type (type SYMBOL) (default object))
)

(deftemplate domain-predicate
  "A predicate symbol in the domain. If a predicate exists, it is true,
   otherwise it is false."
  (slot name (type SYMBOL) (default ?NONE))
  (multislot parameters (default (create$)))
)

(deftemplate domain-operator
  "An operator of the domain. This only defines the name of the operator, all
   other properties (parameters, precondition, effects) are defined in separate
   templates."
  (slot name (type SYMBOL))
)

(deftemplate domain-operator-parameter
  "A parameter of an operator. The operator and type slots must refer to the
   names of an existing operator and an existing type respectively."
  (slot name)
  (slot operator (type SYMBOL))
  (slot type (type SYMBOL) (default obj))
)

(deftemplate domain-precondition
  "A (non-atomic) precondition of an operator. Must be either part-of an
   operator or another precondition. Use the name to assign other preconditions
   as part of this precondition. This can currently be a conjunction or a
   negation. If it is a negation, it can have only one sub-condition. If it is
   a conjunction, it can have an arbitrary number of sub-conditions."
  (slot part-of (type SYMBOL))
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot type (type SYMBOL) (allowed-values conjunction negation))
)

(deftemplate domain-atomic-precondition
  "An atomic precondition of an operator. This must always be part-of a
   non-atomic precondition."
  (slot part-of (type SYMBOL))
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot predicate (type SYMBOL))
  (multislot parameters (type SYMBOL))
  (slot grounded (type SYMBOL) (allowed-values no partially yes) (default no))
)

(deftemplate action
  (slot operator (type SYMBOL))
)

(deftemplate domain-grounding
  "A grounding of a single parameter of an operator"
  (slot operator (type SYMBOL))
  (slot parameter (type SYMBOL))
  (slot value)
)

(deftemplate domain-error
  "A fact representing some error in the domain definition."
  (slot error-msg (type STRING))
)

(defrule domain-compute-precondition-operator-membership
  "Check if a precondition is part of an operator."
  ;(operator (name ?op))
  (domain-precondition (name ?parent))
  (precond-is-part-of ?parent ?op)
  (or (domain-precondition (name ?cond))
      (domain-atomic-precondition (name ?cond)))
  (precond-is-part-of ?cond ?parent)
=>
  (assert (precond-is-part-of ?cond ?op))
)

(defrule domain-translate-precond-part-of-slot-to-fact
  "For any precondition that is part-of an operator or a precondition, also
   assert precond-is-part-of."
  (or (domain-precondition (name ?cond) (part-of ?parent))
      (domain-atomic-precondition (name ?cond) (part-of ?parent)))
=>
  (assert (precond-is-part-of ?cond ?parent))
)

(defrule domain-translate-obj-slot-type-to-ordered-fact
  "Translate the slot type of a domain-object into the ordered fact
   obj-is-of-type."
  (domain-object (name ?obj) (type ?type))
=>
  (assert (obj-is-of-type ?obj ?type))
)

(defrule domain-get-transitive-types
  "An object of type t also has each super-type of t as its type."
  (obj-is-of-type ?obj ?type)
  (domain-object-type (name ?type) (super-type ?super-type))
=>
  (assert (obj-is-of-type ?obj ?super-type))
)

(defrule domain-ground-precondition
  "Ground a precondition of an operator."
  (domain-grounding (operator ?op) (parameter ?p) (value ?v))
  ?precond <- (domain-atomic-precondition (name ?precond-name)
                (parameters $?parameters&:(member$ ?p ?parameters)))
  (precond-is-part-of ?precond-name ?op)
=>
  (duplicate ?precond
    (parameters (replace-member$ ?parameters ?v ?p))
    (grounded partially)
  )
)

(deffunction intersect
  "Compute the intersection of two multi-fields."
  (?s1 ?s2)
  (bind ?res (create$))
  (foreach ?m ?s1
    (if (member$ ?m ?s2) then
      (bind ?res (insert$ ?res (+ (length$ ?res) 1) ?m))
    )
  )
  (return ?res)
)

(defrule domain-check-if-grounded
  "Check if a precondition is completely grounded."
  ?instance <- (domain-atomic-precondition (name ?precond-name)
                (parameters $?grounded-params) (grounded partially))
  (domain-atomic-precondition (name ?precond-name) (grounded no)
    (parameters $?params&:
      (eq nil (nth$ 1 (intersect ?grounded-params ?params)))
    )
  )
=>
  (modify ?instance (grounded yes))
)

(defrule domain-preconditions-without-params-are-grounded
  "Special case of the rule above: If the precondition does not have any
   parameters, it is always grounded."
  ?precond <- (domain-atomic-precondition
                (parameters $?params&:(eq nil (nth$ 1 ?params))))
=>
  (duplicate ?precond (grounded yes))
)

(defrule domain-remove-partially-grounded-preconditions
  "After we found all fully grounded preconditions, we can remove partially
   grounded preconditions again."
  ?precond <- (domain-atomic-precondition (grounded partially))
=>
  (retract ?precond)
)

(defrule domain-check-if-atomic-precondition-is-satisfied
  (domain-atomic-precondition
    (name ?precond) (predicate ?pred) (parameters $?params) (grounded yes)
  )
  (domain-predicate (name ?pred) (parameters $?params))
=>
  (assert (is-satisfied ?precond))
)

(defrule domain-check-if-negative-precondition-is-satisfied
  "A negative precondition is satisfied iff its (only) child is not satisfied.
   Note that we need a second rule that retracts the fact if the child is
   asserted."
  (domain-precondition (name ?precond) (type negation))
  (or (domain-atomic-precondition (name ?child) (part-of ?precond))
      (domain-precondition (name ?child) (part-of ?precond)))
  (not (is-satisfied ?child))
=>
  (assert (is-satisfied ?precond))
)

(defrule domain-retract-negative-precondition-if-child-is-satisfied
  "If a negative precondition's child is satisfied, the precondition is not
   satisfied anymore."
  (domain-precondition (name ?precond) (type negation))
  ?sat <- (is-satisfied ?precond)
  (or (domain-atomic-precondition (name ?child) (part-of ?precond))
      (domain-precondition (name ?child) (part-of ?precond)))
  (is-satisfied ?child)
=>
  (retract ?sat)
)

(defrule domain-check-if-conjunctive-precondition-is-satisfied
  "All the precondition's children must be satisfied."
  (domain-precondition (name ?precond) (type conjunction))
  (not (and (domain-atomic-precondition (part-of ?precond) (name ?child))
            (not (is-satisfied ?child))))
  (not (and (domain-precondition (part-of ?precond) (name ?child))
            (not (is-satisfied ?child))))
=>
  (assert (is-satisfied ?precond))
)

(defrule domain-check-precondition-has-an-operator
  "Check that for each precondition, some operator is defined."
  (domain-precondition (name ?precond))
  (not (precond-is-part-of ?precond ?op))
=>
  (assert (domain-error (error-msg (str-cat "Precondition " ?precond
                                     " does not belong to any operator."))))
)

(defrule domain-check-precondition-belongs-to-existing-operator
  "Check that all defined preconditions belong to a defined operator."
  (domain-precondition (name ?precond))
  (not (and (precond-is-part-of ?precond ?op)
            (domain-operator (name ?op))))
=>
  (assert (domain-error (error-msg (str-cat "Precondition " ?precond
                                     " is part of a non-existing operator.")
  )))
)

(defrule domain-check-object-types-exist
  "Make sure that each specified type of an object actually exists."
  (domain-object (name ?obj) (type ?type))
  (not (domain-object-type (name ?type)))
=>
  (assert (domain-error (error-msg (str-cat "Type " ?type " of object " ?obj
                                     " does not exist."))))
)

(defrule domain-check-super-type-exists
  "Make sure that a super-type of any type in the domain actually exists."
  (domain-object-type (name ?type) (super-type ?super-type))
  (not (domain-object-type (name ?super-type)))
=>
  (assert (domain-error (error-msg (str-cat "Super-type " ?super-type
                                    " of type " ?type " does not exist."))))
)

(deffacts domain-facts
  (domain-object-type (name object))
)
