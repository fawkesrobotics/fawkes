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
  ; If the predicate is sensed, it is not directly changed by an action effect.
  ; Instead, we expect the predicate to be changed externally.
  (slot sensed (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  ; A value predicate is a predicate that is true for at most one value of the
  ; last argument. In other words, the predicate represents a partial function,
  ; with all but the last predicate argument being the function paramters, and
  ; the last predicate being the function value.
  (slot value-predicate (type SYMBOL) (allowed-values TRUE FALSE)
    (default FALSE))
  (multislot param-names (type SYMBOL))
  (multislot param-types (type SYMBOL))
)

(deftemplate domain-fact
  "An instantiated predicate (fact) according to a domain predicate spec.
   If a fact exists, it is considered to be true, false otherwise (closed world assumption)."
  (slot name (type SYMBOL) (default ?NONE))
  (multislot param-values)
)

(deffunction domain-wipe ()
	(foreach ?t (create$ domain-object-type domain-object domain-predicate domain-fact
											 domain-precondition domain-atomic-precondition
											 domain-operator domain-operator-parameter
											 domain-effect domain-error)
		(delayed-do-for-all-facts ((?d ?t)) TRUE (retract ?d))
	)
)

(deffunction domain-fact-key (?name ?param-names ?param-values)
	(if (<> (length$ ?param-names) (length$ ?param-values)) then
		(printout error "Cannot generate domain fact key with non-equal length names and values" crlf)
		(return FALSE)
	)
	(bind ?rv (create$ ?name))
	(if (> (length$ ?param-names) 0) then
		(bind ?rv (append$ ?rv args?))
		(foreach ?n ?param-names (bind ?rv (append$ ?rv ?n (nth$ ?n-index ?param-values))))
	)
	(return ?rv)
)

(deffunction domain-fact-args (?param-names ?param-values)
	(if (<> (length$ ?param-names) (length$ ?param-values)) then
		(printout error "Cannot generate domain fact args with non-equal length names and values" crlf)
		(return FALSE)
	)
	(bind ?args (create$))
	(if (> (length$ ?param-names) 0) then
		(bind ?args (append$ ?args args?))
		(foreach ?n ?param-names (bind ?args (append$ ?args ?n (nth$ ?n-index ?param-values))))
	)
	(return ?args)
)

(deftemplate domain-pending-sensed-fact
  "An action effect of a sensed predicate that is still pending."
  (slot name (type SYMBOL) (default ?NONE))
  (slot goal-id (type SYMBOL))
  (slot plan-id (type SYMBOL))
  ; TODO: Rename to action for consistency. Do this when we no longer need to
  ; stay compatible with lab course code.
  (slot action-id (type INTEGER))
  (slot type (type SYMBOL) (allowed-values POSITIVE NEGATIVE)
    (default POSITIVE))
  (multislot param-values)
)


(deftemplate domain-operator
  "An operator of the domain. This only defines the name of the operator,
   other properties such as parameters, precondition, or effects are
   defined in separate templates.
   The wait-sensed slot defines whether to wait for sensed predicates to
   achieve the desired value, or whether to ignore such predicates."
  (slot name (type SYMBOL))
  (multislot param-names)
	(slot wait-sensed (type SYMBOL) (allowed-values TRUE FALSE) (default TRUE))
  (slot exogenous (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deftemplate domain-operator-parameter
  "A parameter of an operator. The operator and type slots must refer to the
   names of an existing operator and an existing type respectively."
  (slot name)
  (slot operator (type SYMBOL))
  (slot type (type SYMBOL) (default object))
)

(deftemplate domain-precondition
  "A (non-atomic) precondition of an operator or a conditional effect.
   Must be either part-of an operator or another precondition. Use the name to
   assign other preconditions as part of this precondition. This can currently
   be a conjunction or a negation. If it is a negation, it can have only one
   sub-condition. If it is a conjunction, it can have an arbitrary number of
   sub-conditions. The action is an optional ID of grounded action this
   precondition belongs to. Note that grounded should always be yes if the
   action is not nil."
  (slot operator (type SYMBOL))
  (slot part-of (type SYMBOL))
  (slot goal-id (type SYMBOL))
  (slot plan-id (type SYMBOL))
  ; TODO: Rename to action for consistency. Do this when we no longer need to
  ; stay compatible with lab course code.
  (slot grounded-with (type INTEGER) (default 0))
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot type (type SYMBOL) (allowed-values conjunction disjunction negation))
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
  (slot operator (type SYMBOL))
  (slot part-of (type SYMBOL))
  (slot goal-id (type SYMBOL))
  (slot plan-id (type SYMBOL))
  ; TODO: Rename to action for consistency. Do this when we no longer need to
  ; stay compatible with lab course code.
  (slot grounded-with (type INTEGER) (default 0))
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot predicate (type SYMBOL))
  (slot equality (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (multislot param-names (type SYMBOL))
  (multislot param-values (default (create$)))
  (multislot param-constants (default (create$)))
  (slot grounded (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (slot is-satisfied (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deftemplate domain-effect
  "An effect of an operator. For now, effects are just a set of atomic effects
   which are applied after the action was executed successfully."
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
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
   domain-obj-is-of-type."
  (domain-object (name ?obj) (type ?type))
=>
  (assert (domain-obj-is-of-type ?obj ?type))
)

(defrule domain-get-transitive-types
  "An object of type t also has each super-type of t as its type."
  (domain-obj-is-of-type ?obj ?type)
  (domain-object-type (name ?type&~object) (super-type ?super-type))
  (not (domain-obj-is-of-type ?obj ?super-type))
=>
  (assert (domain-obj-is-of-type ?obj ?super-type))
)

(defrule domain-amend-action-params
  "If a plan action has no action-params specified, copy the params from the
   operator."
  ?a <- (plan-action
          (action-name ?op-name)
          (param-names $?ap-names&:(= (length$ ?ap-names) 0))
          (param-values $?ap-values&:(> (length$ ?ap-values) 0))
        )
  ?op <- (domain-operator (name ?op-name)
          (param-names $?param-names&:(> (length$ ?param-names) 0)))
  =>
  (modify ?a (param-names ?param-names))
)

(defrule domain-add-operator-to-top-precondition
  "If a precondition has no operator but is part of a name that is an operator,
   add that name as operator to the precondition."
  ?precond <- (domain-precondition (operator nil) (part-of ?op))
  (domain-operator (name ?op))
=>
  (modify ?precond (operator ?op))
)

(defrule domain-add-operator-to-nested-precondition
  "If a precondition does not have an operator but the parent has one, then copy
   the operator from the parent to the child."
  ?precond <- (domain-precondition (operator nil) (part-of ?parent))
  (domain-precondition (name ?parent) (operator ?op&~nil))
=>
  (modify ?precond (operator ?op))
)

(defrule domain-add-operator-to-atomic-precondition
  "If an atomic precondition does not have an operator but the parent has one,
   then copy the operator from the parent to the child."
  ?precond <- (domain-atomic-precondition (operator nil) (part-of ?parent))
  (domain-precondition (name ?parent) (operator ?op&~nil))
=>
  (modify ?precond (operator ?op))
)

(deffunction remove-precondition
  "Remove an atomic precondition from its parent and clean up the precondition
   tree. If the parent is a disjunction with no other disjunct, simplify it to
   true by removing it recursively. If it is a negation, remove it recursively.
   If it's a conjunction, only remove the conjunct."
  (?precond-name)
  (do-for-fact
    ((?precond domain-atomic-precondition) (?parent domain-precondition))
    (and (eq ?precond:name ?precond-name) (eq ?precond:part-of ?parent:name))
    (if (or (eq ?parent:type disjunction) (eq ?parent:type negation)) then
      (remove-precondition ?parent:name)
    )
    (retract ?precond)
  )
)

(deffunction domain-retract-grounding
  "Retract all ground preconditions."
  ()
  (do-for-all-facts ((?precond domain-precondition))
                    (eq ?precond:grounded TRUE)
                    (retract ?precond))
  (do-for-all-facts ((?precond domain-atomic-precondition))
                    (eq ?precond:grounded TRUE)
                    (retract ?precond))
)


(deffunction domain-is-precond-negative
  "Check if a non-atomic precondition is negative by checking all its parents
   and counting the number of negations. If the number is odd, the precondition
   is negative, otherwise it's positive."
  (?precond-name)
  (do-for-fact
    ((?precond domain-precondition))
    (eq ?precond:name ?precond-name)
    (if (any-factp ((?op domain-operator)) (eq ?op:name ?precond:part-of)) then
      return (eq ?precond:type negation)
    )
    (bind ?parent-is-negative (domain-is-precond-negative ?precond:part-of))
    (return (neq (eq ?precond:type negation) ?parent-is-negative))
  )
)

(defrule domain-remove-precond-on-sensed-nonval-effect-of-exog-action
  "If an exogenous action has a precondition for a non-value predicate that is
   also a sensed effect of the operator, then remove the precondition on the
   effect. This means that part of the exogenous action may already have
   occurred before the action is selected."
  (domain-operator (name ?op) (exogenous TRUE))
  (domain-predicate (name ?pred) (sensed TRUE) (value-predicate FALSE))
  (domain-effect (part-of ?op) (predicate ?pred)
    (param-names $?params) (param-constants $?constants))
  (domain-atomic-precondition (name ?precond) (operator ?op) (grounded FALSE)
    (predicate ?pred) (param-names $?params) (param-constants $?constants))
=>
  (remove-precondition ?precond)
  ; If there are any grounded preconditions, we need to recompute them.
  (domain-retract-grounding)
)

(defrule domain-replace-precond-on-sensed-val-effect-of-exog-action
  "If an exogenous action has a precondition for a value predicate that is also
   a sensed effect of the operator, then remove the precondition on the effect.
   This means that part of the exogenous action may already have occurred before
   the action is selected."
  (domain-operator (name ?op) (exogenous TRUE))
  (domain-predicate (name ?pred) (sensed TRUE) (value-predicate TRUE))
  (domain-effect (part-of ?op) (predicate ?pred) (type POSITIVE)
    (param-names $?args ?eff-val) (param-constants $?const-args ?const-eff-val))
  ?ap <- (domain-atomic-precondition (name ?precond) (operator ?op)
          (part-of ?parent) (predicate ?pred) (grounded FALSE)
          (param-names $?args ?cond-val)
          (param-constants $?const-args ?const-cond-val &:
            (or
                ; The values are constants and the constants are different.
                (and (eq (length$ ?const-args) (length$ ?args))
                     (neq ?const-cond-val ?const-eff-val)
                )
                ; The values are not constants and the parameters are different.
                ; We rely on the fact that the parameter name of constants is
                ; always set to the same value, so this is never satisfied if
                ; the parameters are constants.
                (neq ?cond-val ?eff-val)))
         )
  (not (and (domain-precondition (type disjunction) (name ?parent))
            (domain-atomic-precondition (part-of ?parent) (predicate ?pred)
                (param-names $?args ?eff-val)
                (param-constants $?const-args ?const-eff-val))))
=>
  ; Replace the atomic precondition by a disjunction and add the atomic
  ; precondition as a disjunct. Add the effect as another disjunct.
  (assert (domain-precondition (type disjunction) (name ?precond) (operator ?op)
            (part-of ?parent)))
  (assert (domain-atomic-precondition (name (sym-cat ?precond 2)) (operator ?op)
            (part-of ?precond) (predicate ?pred) (param-names $?args ?eff-val)
            (param-constants $?const-args ?const-eff-val)))
  (modify ?ap (part-of ?precond) (name (sym-cat ?precond 1)))
  ; If there are any grounded preconditions, we need to recompute them.
  (domain-retract-grounding)
)

(defrule domain-ground-action-precondition
  "Ground a non-atomic precondition. Grounding here merely means that we
   duplicate the precondition and tie it to one specific action-id."
  (declare (salience ?*SALIENCE-DOMAIN-GROUND*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  (plan-action (action-name ?op) (goal-id ?g) (plan-id ?p) (id ?action-id)
    (status FORMULATED|PENDING|WAITING))
  ?precond <- (domain-precondition
                (name ?precond-name)
                (part-of ?op)
                (grounded FALSE))
  (not (domain-precondition (name ?precond-name) (goal-id ?g) (plan-id ?p)
        (grounded-with ?action-id) (grounded TRUE)))
=>
  (duplicate ?precond
    (goal-id ?g) (plan-id ?p) (grounded-with ?action-id)
    (grounded TRUE))
)

(defrule domain-ground-effect-precondition
  "Ground a non-atomic precondition. Grounding here merely means that we
   duplicate the precondition and tie it to one specific effect-id."
  (declare (salience ?*SALIENCE-DOMAIN-GROUND*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  (plan-action (action-name ?op) (id ?action-id) (goal-id ?g) (plan-id ?p)
    (status EXECUTION-SUCCEEDED))
  (domain-effect (name ?effect-name) (part-of ?op))
  ?precond <- (domain-precondition
                (name ?precond-name)
                (part-of ?effect-name)
                (grounded FALSE))
  (not (domain-precondition (name ?precond-name) (goal-id ?g) (plan-id ?p)
         (grounded-with ?action-id) (grounded TRUE)))
=>
  (duplicate ?precond (goal-id ?g) (plan-id ?p) (grounded-with ?action-id)
    (grounded TRUE))
)

(defrule domain-ground-nested-precondition
  "Ground a non-atomic precondition that is part of another precondition. Copy
   the action ID from the parent precondition."
  (declare (salience ?*SALIENCE-DOMAIN-GROUND*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?precond <- (domain-precondition
                (name ?precond-name)
                (part-of ?parent)
                (grounded FALSE))
  (domain-precondition (name ?parent) (goal-id ?g) (plan-id ?p)
    (grounded-with ?action-id&~0))
  (not (domain-precondition
        (name ?precond-name)
        (goal-id ?g)
        (plan-id ?p)
        (grounded-with ?action-id)
        (grounded TRUE)))
=>
  (duplicate ?precond (goal-id ?g) (plan-id ?p) (grounded-with ?action-id)
    (grounded TRUE))
)

(defrule domain-ground-atomic-precondition
  "Ground an atomic precondition of an operator."
  (declare (salience ?*SALIENCE-DOMAIN-GROUND*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  (plan-action
    (action-name ?op)
    (goal-id ?g)
    (plan-id ?p)	
    (param-names $?action-param-names)
    (id ?action-id)
    (param-values $?action-values& :
      (= (length$ ?action-values) (length$ ?action-param-names)))
  )
  (domain-precondition (name ?parent) (goal-id ?g) (plan-id ?p)
    (grounded-with ?action-id&~0) (grounded TRUE))
  ?precond <- (domain-atomic-precondition
                (part-of ?parent)
                (name ?precond-name)
                (param-names $?precond-param-names)
                (param-constants $?precond-param-constants)
                (grounded FALSE)
              )
  (not (domain-atomic-precondition
        (goal-id ?g)
        (plan-id ?p)
        (grounded-with ?action-id)
        (name ?precond-name)
        (grounded TRUE)))
=>
  (bind ?values (create$))
  (foreach ?pre ?precond-param-names
    (if (neq (nth$ ?pre-index ?precond-param-constants) nil) then
      (bind ?values
        (insert$ ?values ?pre-index (nth$ ?pre-index ?precond-param-constants)))
    else
      (bind ?action-index (member$ ?pre ?action-param-names))
      (if (not ?action-index) then
        ; ?p is not in the list of the action parameters
        (assert (domain-error (error-type unknown-parameter) (error-msg
          (str-cat "Precondition " ?precond-name " has unknown parameter " ?pre)))
        )
      else
        (bind ?values
          (insert$ ?values ?pre-index (nth$ ?action-index ?action-values)))
      )
    )
  )
  (duplicate ?precond
    (param-values ?values) (goal-id ?g) (plan-id ?p)
    (grounded-with ?action-id) (grounded TRUE))
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
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?precond <- (domain-atomic-precondition
                (goal-id ?g) (plan-id ?p)
                (is-satisfied FALSE)
                (predicate ?pred)
                (equality FALSE)
                (param-values $?params)
                (grounded TRUE))
  (domain-fact (name ?pred) (param-values $?params))
=>
  (modify ?precond (is-satisfied TRUE))
)

(defrule domain-check-if-atomic-equality-precondition-is-satisfied
  ?precond <- (domain-atomic-precondition
                (goal-id ?g) (plan-id ?p)
                (is-satisfied ?is-sat)
                (equality TRUE)
                (param-values $?params& :
                  (and (= (length$ ?params) 2)
                       (neq ?is-sat (eq (nth$ 1 ?params) (nth$ 2 ?params))))
                )
                (grounded TRUE))
=>
  (modify ?precond (is-satisfied (eq (nth$ 1 ?params) (nth$ 2 ?params))))
)

(defrule domain-retract-atomic-precondition-if-not-satisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?precond <- (domain-atomic-precondition
                (goal-id ?g) (plan-id ?p)
                (is-satisfied TRUE)
                (predicate ?pred)
                (param-values $?params)
                (grounded TRUE))
  (not (domain-fact (name ?pred) (param-values $?params)))
=>
  (modify ?precond (is-satisfied FALSE))
)

(defrule domain-check-if-negative-precondition-is-satisfied
  "A negative precondition is satisfied iff its (only) child is not satisfied.
   Note that we need a second rule that retracts the fact if the child is
   asserted."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?precond <- (domain-precondition
                (type negation)
                (grounded TRUE)
                (goal-id ?g)
                (plan-id ?p)
                (grounded-with ?action-id)
                (name ?pn)
                (is-satisfied FALSE))
  (or (domain-atomic-precondition
        (goal-id ?g) (plan-id ?p)
        (grounded-with ?action-id) (part-of ?pn)
        (grounded TRUE) (is-satisfied FALSE)
      )
      (domain-precondition
        (goal-id ?g) (plan-id ?p)
        (grounded-with ?action-id) (part-of ?pn)
        (grounded TRUE) (is-satisfied FALSE)
      )
  )
=>
  (modify ?precond (is-satisfied TRUE))
)

(defrule domain-retract-negative-precondition-if-child-is-satisfied
  "If a negative precondition's child is satisfied, the precondition is not
   satisfied anymore."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?precond <- (domain-precondition
                (type negation)
                (name ?pn)
                (goal-id ?g)
                (plan-id ?p)
                (grounded-with ?action-id)
                (is-satisfied TRUE)
                (grounded TRUE))
  (or (domain-atomic-precondition
        (goal-id ?g) (plan-id ?p)
        (grounded-with ?action-id) (part-of ?pn)
        (grounded TRUE) (is-satisfied TRUE)
      )
      (domain-precondition
        (goal-id ?g) (plan-id ?p)
        (grounded-with ?action-id) (part-of ?pn)
        (grounded TRUE) (is-satisfied TRUE)
      )
  )
=>
  (modify ?precond (is-satisfied FALSE))
)

(defrule domain-check-if-conjunctive-precondition-is-satisfied
  "All the precondition's children must be satisfied."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?precond <- (domain-precondition
                (name ?pn)
                (type conjunction)
                (goal-id ?g)
                (plan-id ?p)
                (grounded-with ?action-id)
                (grounded TRUE)
                (is-satisfied FALSE))
  (not (domain-atomic-precondition
        (goal-id ?g) (plan-id ?p)
        (part-of ?pn) (grounded TRUE)
        (grounded-with ?action-id) (is-satisfied FALSE)))
  (not (domain-precondition
        (goal-id ?g) (plan-id ?p)
        (part-of ?pn) (grounded TRUE)
        (grounded-with ?action-id) (is-satisfied FALSE)))
=>
  (modify ?precond (is-satisfied TRUE))
)

(defrule domain-retract-conjunctive-precondition-if-child-is-not-satisfied
  "Make sure that a conjunctive precondition is not satisfied if any of its
   children is satisfied."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?precond <- (domain-precondition
                (name ?pn)
                (type conjunction)
                (goal-id ?g)
                (plan-id ?p)
                (grounded-with ?action-id)
                (grounded TRUE)
                (is-satisfied TRUE))
  (or (domain-atomic-precondition
        (goal-id ?g) (plan-id ?p)
        (part-of ?pn) (grounded TRUE)
        (grounded-with ?action-id) (is-satisfied FALSE)
      )
      (domain-precondition
        (goal-id ?g) (plan-id ?p)
        (part-of ?pn) (grounded TRUE)
        (grounded-with ?action-id) (is-satisfied FALSE)
      )
  )
=>
  (modify ?precond (is-satisfied FALSE))
)

(defrule domain-check-if-disjunctive-precondition-is-satisfied
  "Check a grounded disjunctive precondition. At least one child must be
   satisfied."
  ?precond <- (domain-precondition
                (name ?pn)
                (type disjunction)
                (goal-id ?g)
                (plan-id ?p)
                (grounded-with ?action-id)
                (grounded TRUE)
                (is-satisfied FALSE))
  (or (domain-atomic-precondition
        (goal-id ?g) (plan-id ?p)
        (part-of ?pn) (grounded TRUE)
        (grounded-with ?action-id) (is-satisfied TRUE))
      (domain-precondition
        (goal-id ?g) (plan-id ?p)
        (part-of ?pn) (grounded TRUE)
        (grounded-with ?action-id) (is-satisfied TRUE))
  )
=>
  (modify ?precond (is-satisfied TRUE))
)

(defrule domain-retract-disjunctive-precondition-if-child-is-not-satisfied
  "If a disjunctive precondition is satisfied but none of its children are, then
   set it to not satisfied."
  ?precond <- (domain-precondition
                (name ?pn)
                (type disjunction)
                (goal-id ?g)
                (plan-id ?p)
                (grounded-with ?action-id)
                (grounded TRUE)
                (is-satisfied TRUE))
  (not
    (or (domain-atomic-precondition
          (goal-id ?g) (plan-id ?p)
          (part-of ?pn) (grounded TRUE)
          (grounded-with ?action-id) (is-satisfied TRUE))
        (domain-precondition
          (goal-id ?g) (plan-id ?p)
          (part-of ?pn) (grounded TRUE)
          (grounded-with ?action-id) (is-satisfied TRUE))
    )
  )
=>
  (modify ?precond (is-satisfied FALSE))
)

(deffunction domain-ground-effect
  "Ground action effect parameters by replacing them with constants and values."
  (?effect-param-names ?effect-param-constants ?action-param-names ?action-param-values)
  (bind ?values $?effect-param-names)
  ; Replace constants with their values
  (foreach ?p ?values
    (if (neq (nth$ ?p-index ?effect-param-constants) nil) then
      (bind ?values
        (replace$ ?values ?p-index ?p-index
          (nth$ ?p-index $?effect-param-constants))
      )
    )
  )
  (foreach ?p $?action-param-names
    (bind ?values
      (replace-member$ ?values (nth$ ?p-index $?action-param-values) ?p)
    )
  )
  (return ?values)
)

; TODO: ?action-name should be ?op
(defrule domain-effects-check-for-sensed
  "Apply effects of an action after it succeeded."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?pa <- (plan-action	(id ?id) (goal-id ?g) (plan-id ?p) (action-name ?op)
                      (status EXECUTION-SUCCEEDED)
											(param-names $?action-param-names)
                      (param-values $?action-param-values))
	(domain-operator (name ?op) (wait-sensed TRUE))
	=>
	(bind ?next-state SENSED-EFFECTS-HOLD)
	(do-for-all-facts ((?e domain-effect) (?pred domain-predicate))
		(and ?pred:sensed (eq ?e:part-of ?op) (eq ?e:predicate ?pred:name))
		; apply if this effect is unconditional or the condition is satisfied
		(if (or (not (any-factp ((?cep domain-precondition)) (eq ?cep:part-of ?e:name)))
						(any-factp ((?cep domain-precondition))
											 (and (eq ?cep:part-of ?e:name) ?cep:is-satisfied
                            (eq ?cep:goal-id ?g) (eq ?cep:plan-id ?p)
                            ?cep:grounded (eq ?cep:grounded-with ?id))))
		 then
			(bind ?values
						(domain-ground-effect ?e:param-names ?e:param-constants
																	?action-param-names ?action-param-values))

			(assert (domain-pending-sensed-fact (name ?pred:name) (action-id ?id) (goal-id ?g) (plan-id ?p)
																					(param-values ?values) (type ?e:type)))
			(bind ?next-state SENSED-EFFECTS-WAIT)
		)
	)
	(modify ?pa (status ?next-state))
)

(defrule domain-effects-ignore-sensed
  "Apply effects of an action after it succeeded."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?pa <- (plan-action	(id ?id) (action-name ?op) (status EXECUTION-SUCCEEDED))
	(domain-operator (name ?op) (wait-sensed FALSE))
	=>
	(modify ?pa (status SENSED-EFFECTS-HOLD))
)

; Atomically assert all effects of an action after it has been executed.
(defrule domain-effects-apply
  "Apply effects of an action after it succeeded."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?pa <- (plan-action	(id ?id) (goal-id ?g) (plan-id ?p) (action-name ?op)
                      (status SENSED-EFFECTS-HOLD)
											(param-names $?action-param-names)
                      (param-values $?action-param-values))
	(domain-operator (name ?op))
	=>
	(do-for-all-facts ((?e domain-effect) (?pred domain-predicate))
		(and (not ?pred:sensed) (eq ?e:part-of ?op) (eq ?e:predicate ?pred:name))

		; apply if this effect is unconditional or the condition is satisfied
		(if (or (not (any-factp ((?cep domain-precondition)) (eq ?cep:part-of ?e:name)))
						(any-factp ((?cep domain-precondition))
											 (and (eq ?cep:part-of ?e:name) ?cep:is-satisfied
                            (eq ?cep:goal-id ?g) (eq ?cep:plan-id ?p)
                            ?cep:grounded (eq ?cep:grounded-with ?id))))
		 then
			(bind ?values
						(domain-ground-effect ?e:param-names ?e:param-constants
																	?action-param-names ?action-param-values))

			(if (eq ?e:type POSITIVE)
			 then
				(assert (domain-fact (name ?pred:name) (param-values ?values)))
			 else
        ; Check if there is also a positive effect for the predicate with the
        ; same values. Only apply the negative effect if no such effect
        ; exists.
        ; NOTE: This does NOT work for conditional effects.
        (if (not (any-factp
                  ((?oe domain-effect))
                  (and
                    (eq ?oe:part-of ?op) (eq ?oe:predicate ?pred:name)
                    (eq ?oe:type POSITIVE)
                    (eq ?values
                        (domain-ground-effect ?oe:param-names
                          ?oe:param-constants ?action-param-names
                          ?action-param-values))
                  )))
        then
				  (delayed-do-for-all-facts ((?df domain-fact))
					  (and (eq ?df:name ?pred:name) (eq ?df:param-values ?values))
					  (retract ?df)
          )
        )
			)
		)
	)
	(modify ?pa (status EFFECTS-APPLIED))
)

(defrule domain-effect-sensed-positive-holds
  "Remove a pending sensed positive fact that has been sensed."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?ef <- (domain-pending-sensed-fact (type POSITIVE)
          (name ?predicate) (param-values $?values))
  ?df <- (domain-fact (name ?predicate) (param-values $?values))
=>
  (retract ?ef)
)

(defrule domain-effect-sensed-negative-holds
  "Remove a pending sensed negative fact that has been sensed."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?ef <- (domain-pending-sensed-fact (type NEGATIVE)
          (name ?predicate) (param-values $?values))
  (not (domain-fact (name ?predicate) (param-values $?values)))
=>
  (retract ?ef)
)

(defrule domain-effect-wait-sensed-done
  "After the effects of an action have been applied, change it to SENSED-EFFECTS-HOLD."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?a <- (plan-action (id ?action-id) (status SENSED-EFFECTS-WAIT) (plan-id ?p) (goal-id ?g))
  (not (domain-pending-sensed-fact (action-id ?action-id) (goal-id ?g) (plan-id ?p)))
  =>
  (modify ?a (status SENSED-EFFECTS-HOLD))
)

(defrule domain-effect-sensed-remove-on-removed-action
  "Remove domain-pending-sensed-fact when the corresponding action was removed"
  ?ef <- (domain-pending-sensed-fact (action-id ?action-id) (goal-id ?g) (plan-id ?p))
  (not (plan-action (id ?action-id) (plan-id ?p) (goal-id ?g)))
  =>
  (retract ?ef)
)

(defrule domain-action-final
  "After the effects of an action have been applied, change it to FINAL."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?a <- (plan-action (id ?action-id) (status EFFECTS-APPLIED))
  =>
  (modify ?a (status FINAL))
  (domain-retract-grounding)
)

; This might be extended: if an action failed, but still all effects
; have been achieved, consider execution to have succeeded.  This
; might happen if all effects only refer to sensed predicates and
; these have the expected values (possibly after a short stabilization
; period).
(defrule domain-action-failed
  "An action has failed."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?a <- (plan-action (id ?action-id) (status EXECUTION-FAILED))
  =>
  (modify ?a (status FAILED))
  (domain-retract-grounding)
)

(defrule domain-check-if-action-is-executable
  "If the precondition of an action is satisfied, the action is executable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?action <- (plan-action (id ?action-id) (goal-id ?g) (plan-id ?p)
                          (action-name ?op) (executable FALSE))
  (domain-precondition (plan-id ?p) (goal-id ?g) (grounded-with ?action-id)
                       (part-of ?op)  (is-satisfied TRUE))
=>
  (modify ?action (executable TRUE))
)

(defrule domain-check-if-action-is-executable-without-precondition
  "If the precondition of an action is satisfied, the action is executable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?action <- (plan-action (id ?action-id) (action-name ?action-name) (executable FALSE))
  (not (domain-precondition (part-of ?action-name)))
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
  (domain-object-type (name ?type) (super-type ?super-type&~object))
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

(defrule domain-check-atomic-precondition-predicate-has-no-equality
  "Make sure that any atomic precondition with a set predicate has equality set
   to FALSE."
  (domain-atomic-precondition
    (name ?precond)
    (predicate ?predicate&~nil)
    (equality TRUE)
  )
=>
  (assert (domain-error
    (error-type precondition-with-equality-and-predicate)
    (error-msg (str-cat "Precondition " ?precond " cannot be an equality"
                        " condition and a condition on the predicate "
                        ?predicate " at the same time"))))
)

(defrule domain-check-atomic-precondition-is-on-predicate-or-equality
  "Make sure that all preconditions have a predicate or are set to equality."
  (domain-atomic-precondition
    (name ?precond)
    (predicate nil)
    (equality FALSE)
  )
=>
  (assert (domain-error
    (error-type precondition-must-have-predicate-or-be-equality)
    (error-msg (str-cat "Precondition " ?precond " must have a predicate "
                        "or set to equality"))))
)

(defrule domain-check-equality-must-have-exactly-two-parameters
  "Make sure that equalities always have exactly two parameters."
  (domain-atomic-precondition
    (name ?precond)
    (equality TRUE)
    (param-names $?param-names &: (neq (length$ ?param-names) 2))
  )
=>
  (assert (domain-error
    (error-type equality-must-have-exactly-two-parameters)
    (error-msg (str-cat "Precondition " ?precond " is an equality precondition"
                        " but has " (length$ ?param-names) " parameters,"
                        " should be 2."))))
)

(defrule domain-check-value-predicate-must-have-only-one-value
  "Make sure that each value predicate has at most one value."
  (domain-predicate (value-predicate TRUE) (name ?pred))
  (domain-fact (name ?pred) (param-values $?args ?val))
  (domain-fact (name ?pred) (param-values $?args ?other-val&~?val))
=>
  (assert (domain-error (error-type value-predicate-with-multiple-values)
    (error-msg (str-cat "Value predicate " ?pred "(" (implode$ ?args) ") "
    "has multiple values " "(" ?val ", " ?other-val ")"))))
)

(defrule domain-check-value-predicate-clean-up-unique-value-error
  "Clean up the error if a value predicate no longer has multiple values."
  ?e <- (domain-error (error-type value-predicate-with-multiple-values))
  (not (and (domain-fact (name ?pred) (param-values $?args ?val))
            (domain-fact (name ?pred) (param-values $?args ?other-val&~?val))))
=>
  (retract ?e)
)

(defrule domain-check-effects-on-value-predicates-must-occur-in-pairs
  "Value predicates can only have exactly one value. Thus, any effect on value
   predicated must occur in pairs."
  (domain-predicate (value-predicate TRUE) (name ?pred))
  (domain-effect (name ?n) (part-of ?op) (predicate ?pred)
    (param-names $?args ?) (type ?type))
  (not (domain-effect (part-of ?op) (predicate ?pred) (param-names $?args ?)
        (type ?other-type&~?type)))
=>
  (assert (domain-error (error-type value-predicate-without-paired-effect)
            (error-msg (str-cat "Effect " ?n " of operator " ?op " on " ?pred
              " (" (implode$ ?args) ") "
              "is not matched with a complementary effect"))))
)

(defrule domain-print-error
  (domain-error (error-type ?type) (error-msg ?msg))
  =>
  (printout error "Domain error '" ?type "': " ?msg crlf)
)
