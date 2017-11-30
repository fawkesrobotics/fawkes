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
	"Representation of a predicate specification."
  (slot name (type SYMBOL) (default ?NONE))
	(slot wm-key-pattern (type STRING))
  (multislot param-names (type SYMBOL))
  (multislot param-types (type SYMBOL))
)

(deftemplate domain-fact
  "An instantiated predicate (fact) according to a domain predicate spec.
   If a fact exists, it is considered to be true, false otherwise (closed world assumption)."
  (slot name (type SYMBOL) (default ?NONE))
  (multislot param-values)
)

(deftemplate domain-retracted-fact
  "Helper template that is asserted if a predicate is to be retracted."
  (slot name (type SYMBOL) (default ?NONE))
  (multislot param-values)
)


(deftemplate domain-operator
  "An operator of the domain. This only defines the name of the operator, all
   other properties (parameters, precondition, effects) are defined in separate
   templates."
  (slot name (type SYMBOL))
  (multislot param-names)
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
  (multislot param-constants (default (create$)))
  (slot type (type SYMBOL) (allowed-values POSITIVE NEGATIVE)
    (default POSITIVE))
)

(deftemplate domain-error
  "A fact representing some error in the domain definition."
  (slot error-msg (type STRING))
  (slot error-type (type SYMBOL))
)

(defrule domain-add-type-object
  "Make sure we always have a domain object type 'object'."
  (not (domain-object-type (name object)))
  =>
  (assert (domain-object-type (name object)))
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

(defrule domain-amend-action-params
  "If a plan action has no action-params specified, copy the params from the
   operator."
  ?a <- (plan-action
          (action-name ?op-name)
          (param-names $?ap-names&:(= (length$ ?ap-names) 0))
          (param-values $?ap-values&:(> (length$ ?ap-values) 0))
        )
  ?op <- (domain-operator (name ?op-name) (param-names $?param-names))
  =>
  (modify ?a (param-names ?param-names))
)

(defrule domain-ground-precondition
  "Ground a non-atomic precondition. Grounding here merely means that we
   duplicate the precondition and tie it to one specific action-id."
  (not (domain-wm-update))
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
  (not (domain-wm-update))
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
  (not (domain-wm-update))
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
      (if (not ?action-index) then
        ; ?p is not in the list of the action parameters
        (assert (domain-error (error-type unknown-parameter) (error-msg
          (str-cat "Precondition " ?precond-name " has unknown parameter " ?p)))
        )
      else
        (bind ?values
          (insert$ ?values ?p-index (nth$ ?action-index ?action-values)))
      )
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
  (domain-fact (name ?pred) (param-values $?params))
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
                (action ?action-id)
                (name ?pn)
                (is-satisfied FALSE))
  (or (domain-atomic-precondition
        (action ?action-id) (part-of ?pn) (grounded TRUE) (is-satisfied FALSE))
      (domain-precondition
        (action ?action-id) (part-of ?pn) (grounded TRUE) (is-satisfied FALSE)))
=>
  (modify ?precond (is-satisfied TRUE))
)

(defrule domain-retract-negative-precondition-if-child-is-satisfied
  "If a negative precondition's child is satisfied, the precondition is not
   satisfied anymore."
  ?precond <- (domain-precondition
                (type negation)
                (name ?pn)
                (action ?action-id)
                (is-satisfied TRUE)
                (grounded TRUE))
  (or (domain-atomic-precondition
        (action ?action-id) (part-of ?pn) (grounded TRUE) (is-satisfied TRUE))
      (domain-precondition
        (action ?action-id) (part-of ?pn) (grounded TRUE) (is-satisfied TRUE)))
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
    (param-constants $?effect-param-constants)
    (type ?effect-type)
    (predicate ?predicate))
=>
  (bind ?values ?effect-param-names)
  ; Replace constants with their values
  (foreach ?p ?values
    (if (eq ?p c) then
      (bind ?values
        (replace$ ?values ?p-index ?p-index
          (nth$ ?p-index ?effect-param-constants))
      )
    )
  )
  (foreach ?p ?action-param-names
    (bind ?values
      (replace-member$ ?values (nth$ ?p-index ?action-param-values) ?p)
    )
  )
  (if (eq ?effect-type POSITIVE) then
    (assert (domain-fact (name ?predicate) (param-values ?values)))
  else
    (assert (domain-retracted-fact (name ?predicate) (param-values ?values)))
  )
  (assert (domain-wm-update))
)

(defrule domain-retract-negative-effect
  "Retract an existing predicate if the same retracted predicate exists."
  ?p <- (domain-fact (name ?predicate) (param-values $?params))
  (domain-retracted-fact (name ?predicate) (param-values $?params))
=>
  (retract ?p)
)

(defrule domain-cleanup-retract-facts
  "Clean up a retract facts after retracting."
  (declare (salience -100))
  ?r <- (domain-retracted-fact)
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
  (assert (domain-error (error-type precondition-without-parent)
    (error-msg
    (str-cat "Precondition " ?precond
      " does not belong to any operator or precondition."))))
)

(defrule domain-check-object-types-exist
  "Make sure that each specified type of an object actually exists."
  (domain-object (name ?obj) (type ?type))
  (not (domain-object-type (name ?type)))
=>
  (assert (domain-error
    (error-type type-of-object-does-not-exist)
    (error-msg (str-cat "Type " ?type " of object " ?obj " does not exist."))))
)

(defrule domain-check-super-type-exists
  "Make sure that a super-type of any type in the domain actually exists."
  (domain-object-type (name ?type) (super-type ?super-type))
  (not (domain-object-type (name ?super-type)))
=>
  (assert (domain-error (error-type super-type-does-not-exist)
    (error-msg (str-cat "Super-type " ?super-type
                        " of type " ?type " does not exist."))))
)

(defrule domain-check-operator-of-action-exists
  "Make sure that for each action in a plan, the respective operator exists."
  (plan-action (action-name ?op))
  (not (domain-operator (name ?op)))
  =>
  (assert (domain-error (error-type operator-of-action-does-not-exist)
    (error-msg (str-cat "Operator of action " ?op " does not exist"))))
)

(defrule domain-cleanup-preconditions-on-worldmodel-change
  "Retract grounded preconditions when the worldmodel changes."
  (domain-wm-update)
  ?precond <- (domain-precondition (grounded TRUE))
=>
  (retract ?precond)
)

(defrule domain-cleanup-atomic-preconditions-on-worldmodel-change
  "Retract grounded atomic preconditions when the worldmodel changes."
  (domain-wm-update)
  ?precond <- (domain-atomic-precondition (grounded TRUE))
=>
  (retract ?precond)
)

(defrule domain-wm-update-done
  "Updating the world model finished, cleanup update fact."
  ?wmu <- (domain-wm-update)
=>
  (retract ?wmu)
)

(deffacts domain-facts
  (domain-object-type (name object))
)
