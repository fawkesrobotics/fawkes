;---------------------------------------------------------------------------
;  domain.clp - Representation of a planning domain
;
;  Created: Fri 22 Sep 2017 11:35:49 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate obj-type
  "A type in the domain. The type obj must be super-type of all types."
  (slot name (type SYMBOL))
  (slot super-type (type SYMBOL) (default object))
)

(deftemplate dom-object
  "An object in the domain with the given name and type. The type must refer to
   the name of an existing type."
  (slot name)
  (slot obj-type (type SYMBOL) (default object))
)

(deftemplate predicate
  "A predicate symbol in the domain. If a predicate exists, it is true,
   otherwise it is false."
  (slot name (type SYMBOL) (default ?NONE))
  (multislot parameters (default (create$)))
)

(deftemplate operator
  "An operator of the domain. This only defines the name of the operator, all
   other properties (parameters, precondition, effects) are defined in separate
   templates."
  (slot name (type SYMBOL))
)

(deftemplate parameter
  "A parameter of an operator. The operator and obj-type slots must refer to the
   names of an existing operator and an existing type respectively."
  (slot name)
  (slot operator (type SYMBOL))
  (slot obj-type (type SYMBOL) (default obj))
)

(deftemplate precondition
  "A (non-atomic) precondition of an operator. Must be either part-of an
   operator or another precondition. Use the name to assign other preconditions
   as part of this precondition. This can currently be a conjunction or a
   negation. If it is a negation, it can have only one sub-condition. If it is
   a conjunction, it can have an arbitrary number of sub-conditions."
  (slot part-of (type SYMBOL))
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot type (type SYMBOL) (allowed-values conjunction negation))
)

(deftemplate atomic-precondition
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

(deftemplate grounding
  "A grounding of a single parameter of an operator"
  (slot operator (type SYMBOL))
  (slot parameter (type SYMBOL))
  (slot value)
)

(deftemplate domain-error
  "A fact representing some error in the domain definition."
  (slot error-msg (type STRING))
)

(defrule compute-precondition-operator-membership
  "Check if a precondition is part of an operator."
  ;(operator (name ?op))
  (precondition (name ?parent))
  (precond-is-part-of ?parent ?op)
  (or (precondition (name ?cond))
      (atomic-precondition (name ?cond)))
  (precond-is-part-of ?cond ?parent)
=>
  (assert (precond-is-part-of ?cond ?op))
)

(defrule translate-precond-part-of-slot-to-fact
  "For any precondition that is part-of an operator or a precondition, also
   assert precond-is-part-of."
  (or (precondition (name ?cond) (part-of ?parent))
      (atomic-precondition (name ?cond) (part-of ?parent)))
=>
  (assert (precond-is-part-of ?cond ?parent))
)

(defrule translate-obj-slot-type-to-ordered-fact
  "Translate the slot type of a dom-object into the ordered fact
   obj-is-of-type."
  (dom-object (name ?obj) (obj-type ?type))
=>
  (assert (obj-is-of-type ?obj ?type))
)

(defrule get-transitive-types
  "An object of type t also has each super-type of t as its type."
  (obj-is-of-type ?obj ?type)
  (obj-type (name ?type) (super-type ?super-type))
=>
  (assert (obj-is-of-type ?obj ?super-type))
)

(defrule ground-precondition
  "Ground a precondition of an operator."
  (grounding (operator ?op) (parameter ?p) (value ?v))
  ?precond <- (atomic-precondition (name ?precond-name)
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

(defrule check-if-grounded
  "Check if a precondition is completely grounded."
  ?instance <- (atomic-precondition (name ?precond-name)
                (parameters $?grounded-params) (grounded partially))
  (atomic-precondition (name ?precond-name) (grounded no)
    (parameters $?params&:
      (eq nil (nth$ 1 (intersect ?grounded-params ?params)))
    )
  )
=>
  (modify ?instance (grounded yes))
)

(defrule preconditions-without-params-are-grounded
  "Special case of the rule above: If the precondition does not have any
   parameters, it is always grounded."
  ?precond <- (atomic-precondition
                (parameters $?params&:(eq nil (nth$ 1 ?params))))
=>
  (duplicate ?precond (grounded yes))
)

(defrule remove-partially-grounded-preconditions
  "After we found all fully grounded preconditions, we can remove partially
   grounded preconditions again."
  ?precond <- (atomic-precondition (grounded partially))
=>
  (retract ?precond)
)

(defrule check-if-atomic-precondition-is-satisfied
  (atomic-precondition
    (name ?precond) (predicate ?pred) (parameters $?params) (grounded yes)
  )
  (predicate (name ?pred) (parameters $?params))
=>
  (assert (is-satisfied ?precond))
)

(defrule check-if-negative-precondition-is-satisfied
  "A negative precondition is satisfied iff its (only) child is not satisfied.
   Note that we need a second rule that retracts the fact if the child is
   asserted."
  (precondition (name ?precond) (type negation))
  (or (atomic-precondition (name ?child) (part-of ?precond))
      (precondition (name ?child) (part-of ?precond)))
  (not (is-satisfied ?child))
=>
  (assert (is-satisfied ?precond))
)

(defrule retract-negative-precondition-if-child-is-satisfied
  "If a negative precondition's child is satisfied, the precondition is not
   satisfied anymore."
  (precondition (name ?precond) (type negation))
  ?sat <- (is-satisfied ?precond)
  (or (atomic-precondition (name ?child) (part-of ?precond))
      (precondition (name ?child) (part-of ?precond)))
  (is-satisfied ?child)
=>
  (retract ?sat)
)

(defrule check-if-conjunctive-precondition-is-satisfied
  "All the precondition's children must be satisfied."
  (precondition (name ?precond) (type conjunction))
  (not (and (atomic-precondition (part-of ?precond) (name ?child))
            (not (is-satisfied ?child))))
  (not (and (precondition (part-of ?precond) (name ?child))
            (not (is-satisfied ?child))))
=>
  (assert (is-satisfied ?precond))
)

(defrule domain-check-precondition-has-an-operator
  "Check that for each precondition, some operator is defined."
  (precondition (name ?precond))
  (not (precond-is-part-of ?precond ?op))
=>
  (assert (domain-error (error-msg (str-cat "Precondition " ?precond
                                     " does not belong to any operator."))))
)

(defrule domain-check-precondition-belongs-to-existing-operator
  "Check that all defined preconditions belong to a defined operator."
  (precondition (name ?precond))
  (not (and (precond-is-part-of ?precond ?op)
            (operator (name ?op))))
=>
  (assert (domain-error (error-msg (str-cat "Precondition " ?precond
                                     " is part of a non-existing operator.")
  )))
)

(defrule domain-check-object-types-exist
  "Make sure that each specified type of an object actually exists."
  (dom-object (name ?obj) (obj-type ?type))
  (not (obj-type (name ?type)))
=>
  (assert (domain-error (error-msg (str-cat "Type " ?type " of object " ?obj
                                     " does not exist."))))
)

(defrule domain-check-super-type-exists
  "Make sure that a super-type of any type in the domain actually exists."
  (obj-type (name ?type) (super-type ?super-type))
  (not (obj-type (name ?super-type)))
=>
  (assert (domain-error (error-msg (str-cat "Super-type " ?super-type
                                    " of type " ?type " does not exist."))))
)

(deffacts domain-facts
  (obj-type (name object))
)
