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

(deftemplate domain-retracted-predicate
  "Helper template that is asserted if a predicate is to be retracted."
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
   a conjunction, it can have an arbitrary number of sub-conditions. The action
   is an optional ID of grounded action this precondition belongs to. Note that
   grounded should always be yes if the action is not nil."
  (slot part-of (type SYMBOL))
  (slot action (type INTEGER) (default 0))
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot type (type SYMBOL) (allowed-values conjunction negation))
  (slot grounded (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (slot is-satisfied (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deftemplate domain-atomic-precondition
  "An atomic precondition of an operator. This must always be part-of a
   non-atomic precondition. The multislot param-constants can be used to define
   predicate arguments that are not parameters of the operator. After grounding,
   if the ith slot of param-constants contains the value v != nil, then the ith
   slot of param-values will also contain the value v. In that case, the ith
   value of param-names will be ignored and should be set to c (for constant).
   See the tests for an example.
"
  (slot part-of (type SYMBOL))
  (slot action (type INTEGER) (default 0))
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot predicate (type SYMBOL))
  (multislot param-names (type SYMBOL))
  (multislot param-values (default (create$)))
  (multislot param-constants (default (create$)))
  (slot grounded (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (slot is-satisfied (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deftemplate domain-effect
  "An effect of an operator. For now, effects are just a set of atomic effects
   which are applied after the action was executed successfully."
  (slot part-of (type SYMBOL))
  (slot predicate (type SYMBOL))
  (multislot param-names (default (create$)))
  (multislot param-values (default (create$)))
  (slot type (type SYMBOL) (allowed-values POSITIVE NEGATIVE)
    (default POSITIVE))
)

(deftemplate domain-error
  "A fact representing some error in the domain definition."
  (slot error-msg (type STRING))
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
  "Ground a non-atomic precondition. Grounding here merely means that we
   duplicate the precondition and tie it to one specific action-id."
  (plan-action (action-name ?op) (id ?action-id))
  ?precond <- (domain-precondition
                (name ?precond-name)
                (part-of ?op)
                (grounded FALSE))
  (not (domain-precondition
        (name ?precond-name) (action ?action-id) (grounded TRUE)))
=>
  (duplicate ?precond (action ?action-id) (grounded TRUE))
)

(defrule domain-ground-nested-precondition
  "Ground a non-atomic precondition that is part of another precondition. Copy
   the action ID from the parent precondition."
  ?precond <- (domain-precondition
                (name ?precond-name)
                (part-of ?parent)
                (grounded FALSE))
  (domain-precondition (name ?parent) (action ?action-id&~0))
  (not (domain-precondition
        (name ?precond-name)
        (action ?action-id)
        (grounded TRUE)))
=>
  (duplicate ?precond (action ?action-id) (grounded TRUE))
)

(defrule domain-ground-atomic-precondition
  "Ground an atomic precondition of an operator."
  (plan-action
    (action-name ?op)
    (param-names $?action-param-names)
    (id ?action-id)
    (param-values $?action-values))
  (domain-precondition (name ?parent) (action ?action-id&~0) (grounded TRUE))
  ?precond <- (domain-atomic-precondition
                (part-of ?parent)
                (name ?precond-name)
                (param-names $?precond-param-names)
                (param-constants $?precond-param-constants)
                (grounded FALSE)
              )
  (not (domain-atomic-precondition
        (action ?action-id)
        (name ?precond-name)
        (grounded TRUE)))
=>
  (bind ?values (create$))
  (foreach ?p ?precond-param-names
    (if (neq (nth$ ?p-index ?precond-param-constants) nil) then
      (bind ?values
        (insert$ ?values ?p-index (nth$ ?p-index ?precond-param-constants)))
    else
      (bind ?action-index (member$ ?p ?action-param-names))
      (bind ?values
        (insert$ ?values ?p-index (nth$ ?action-index ?action-values)))
    )
  )
  (duplicate ?precond
    (param-values ?values) (action ?action-id) (grounded TRUE))
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

(defrule domain-check-if-atomic-precondition-is-satisfied
  ?precond <- (domain-atomic-precondition
                (is-satisfied FALSE)
                (predicate ?pred)
                (param-values $?params)
                (grounded TRUE))
  (domain-predicate (name ?pred) (parameters $?params))
=>
  (modify ?precond (is-satisfied TRUE))
)

(defrule domain-check-if-negative-precondition-is-satisfied
  "A negative precondition is satisfied iff its (only) child is not satisfied.
   Note that we need a second rule that retracts the fact if the child is
   asserted."
  ?precond <- (domain-precondition
                (type negation)
                (grounded TRUE)
                (name ?pn)
                (is-satisfied FALSE))
  (or (domain-atomic-precondition
        (part-of ?pn) (grounded TRUE) (is-satisfied FALSE))
      (domain-precondition
        (part-of ?pn) (grounded TRUE) (is-satisfied FALSE)))
=>
  (modify ?precond (is-satisfied TRUE))
)

(defrule domain-retract-negative-precondition-if-child-is-satisfied
  "If a negative precondition's child is satisfied, the precondition is not
   satisfied anymore."
  ?precond <- (domain-precondition
                (type negation)
                (name ?pn)
                (is-satisfied TRUE)
                (grounded TRUE))
  (or (domain-atomic-precondition
        (part-of ?pn) (grounded TRUE) (is-satisfied TRUE))
      (domain-precondition
        (part-of ?pn) (grounded TRUE) (is-satisfied TRUE)))
=>
  (modify ?precond (is-satisfied FALSE))
)

(defrule domain-check-if-conjunctive-precondition-is-satisfied
  "All the precondition's children must be satisfied."
  ?precond <- (domain-precondition
                (name ?pn)
                (type conjunction)
                (action ?action-id)
                (grounded TRUE)
                (is-satisfied FALSE))
  (not (domain-atomic-precondition
        (part-of ?pn) (grounded TRUE) (action ?action-id) (is-satisfied FALSE)))
  (not (domain-precondition
        (part-of ?pn) (grounded TRUE) (action ?action-id) (is-satisfied FALSE)))
=>
  (modify ?precond (is-satisfied TRUE))
)

(defrule domain-retract-conjunctive-precondition-if-child-is-not-satisfied
  "Make sure that a conjunctive precondition is not satisfied if any of its
   children is satisfied."
  ?precond <- (domain-precondition
                (name ?pn)
                (type conjunction)
                (action ?action-id)
                (grounded TRUE)
                (is-satisfied TRUE))
  (or (domain-atomic-precondition
        (part-of ?pn) (grounded TRUE) (action ?action-id) (is-satisfied FALSE))
      (domain-precondition
        (part-of ?pn) (grounded TRUE) (action ?action-id) (is-satisfied FALSE))
  )
=>
  (modify ?precond (is-satisfied FALSE))
)

(defrule domain-apply-effect
  "Apply an effect of an action after it succeeded."
  (plan-action
    (id ?id)
    (action-name ?op)
    (status FINAL)
    (param-names $?action-param-names)
    (param-values $?action-param-values)
  )
  (domain-effect
    (part-of ?op)
    (param-names $?effect-param-names)
    (type ?effect-type)
    (predicate ?predicate))
=>
  (bind ?values ?effect-param-names)
  (foreach ?p ?action-param-names
    (bind ?values
      (replace-member$ ?values (nth$ ?p-index ?action-param-values) ?p)
    )
  )
  (if (eq ?effect-type POSITIVE) then
    (assert (domain-predicate (name ?predicate) (parameters ?values)))
  else
    (assert (domain-retracted-predicate (name ?predicate) (parameters ?values)))
  )
)

(defrule domain-retract-negative-effect
  "Retract an existing predicate if the same retracted predicate exists."
  ?p <- (domain-predicate (name ?predicate) (parameters $?params))
  ?r <- (domain-retracted-predicate (name ?predicate) (parameters $?params))
=>
  (retract ?r ?p)
)

(defrule domain-cleanup-retract-facts
  "Clean up a retract-predicate if the respective predicate does not exist."
  ?r <- (domain-retracted-predicate)
=>
  (retract ?r)
)

(defrule domain-check-if-action-is-executable
  "If the precondition of an action is satisfied, the action is executable."
  ?action <- (plan-action (id ?action-id) (executable FALSE))
  (domain-precondition (action ?action-id) (is-satisfied TRUE))
=>
  (modify ?action (executable TRUE))
)

(defrule domain-check-precondition-has-an-operator
  "Check that for each precondition, some operator is defined."
  (domain-precondition (name ?precond) (part-of ?parent))
  (not (domain-precondition (name ?parent)))
  (not (domain-operator (name ?parent)))
=>
  (assert (domain-error (error-msg
    (str-cat "Precondition " ?precond
      " does not belong to any operator or precondition."))))
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
