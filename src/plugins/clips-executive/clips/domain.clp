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
											 pddl-formula pddl-predicate pddl-grounding
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

(deftemplate pddl-grounding
  "A fact storing the grounding information of PDDL formulas/predicates. Used to model
  the relation between the source of the grounding information and the grounded instance
  explicitely."
  (slot id (type SYMBOL) (default ?NONE))

  (multislot param-names (type SYMBOL))
  (multislot param-values (type SYMBOL))
)

(deffunction domain-param-value-for-name
  "Given a list of param names, param values and a specific key, return the
  corresponding value"
  (?param-values ?param-names ?param-name)

  (bind ?pos (member$ ?param-name ?param-names))

  (if ?pos then
    (return (nth$ ?pos ?param-values))
  )

  (return FALSE)
)

(deftemplate pddl-formula
  "A PDDL formula representation in CLIPS, sourced from the preconditions of
  the PDDL domain description."
  (slot id (type SYMBOL) (default ?NONE))
  (slot part-of (type SYMBOL))

  (slot type (type SYMBOL) (allowed-values conjunction disjunction negation atom))
)

(deftemplate grounded-pddl-formula
  "A grounded instance of a PDDL formula. Grounded instances are usually
  associated with one plan-action. Grounding itself only occurs on the
  predicate level."
  (slot id (type SYMBOL) (default ?NONE))
  (slot formula-id (type SYMBOL)); reference to ungrounded base version
  (slot grounding (type SYMBOL))

  (slot is-satisfied (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deftemplate pddl-predicate
  "An instantiated predicate with possible constants that is part of
  a PDDL formula."
  (slot id (type SYMBOL) (default ?NONE))
  (slot part-of (type SYMBOL)) ; reference to parent formula

  ; a PDDL predicate is either an equality or references a domain-predicate
  ; equalities are marked through the symbol 'equality'
  (slot predicate (type SYMBOL))

  (multislot param-names (type SYMBOL))
  (multislot param-constants)
)

(deftemplate grounded-pddl-predicate
  "A grounded instance of a PDDL predicate. Grounded instances have a value for
  each parameter slot and are part of a grounded formula."
  (slot id (type SYMBOL) (default ?NONE))
  (slot predicate-id (type SYMBOL)) ; reference to ungrounded base version
  (slot grounding (type SYMBOL))

  (slot is-satisfied (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deffunction ground-pddl-predicate
  "Ground a PDDL predicate based on the given parameter values from a set of param names
  and param values."
  (?parent-id ?param-names ?param-values ?grounding-id)

  (do-for-all-facts ((?predicate pddl-predicate))
        (eq ?parent-id ?predicate:part-of)

    (assert (grounded-pddl-predicate
          (id (sym-cat "grounded-" ?predicate:id "-" ?grounding-id))
          (predicate-id ?predicate:id)
          (grounding ?grounding-id))
    )
  )
)

(deffunction ground-pddl-formula
  "Ground a PDDL formula recursively based on the given values from a set of param
  names and param values."
  (?parent-id ?param-names ?param-values ?grounding-id)

  (do-for-all-facts ((?formula pddl-formula)) (eq ?parent-id ?formula:part-of)
    ;if no grounding fact created yet, create one
    (if (eq ?grounding-id nil) then
      (bind ?grounding (assert (pddl-grounding (id (sym-cat "grounding-" ?formula:id "-" (gensym*)))
                                               (param-names ?param-names)
                                               (param-values ?param-values))))
      (bind ?grounding-id (fact-slot-value ?grounding id))
    )

    ;recursively ground subformulas
    (bind ?grounded-id (sym-cat "grounded-" ?formula:id "-" (gensym*)))

    (assert (grounded-pddl-formula (formula-id ?formula:id)
                                   (id ?grounded-id)
                                   (grounding ?grounding-id)))

    (ground-pddl-formula ?formula:id ?param-names ?param-values ?grounding-id)
    (ground-pddl-predicate ?formula:id ?param-names ?param-values ?grounding-id)
  )
  (return ?grounding-id)
)

(defrule domain-ground-plan-action-precondition
  "Create grounded pddl formulas for the precondition of a plan-action if it
  has not been grounded yet and add a reference to the plan-action fact"
  (declare (salience ?*SALIENCE-DOMAIN-GROUND*))
  ?p <- (plan-action (id ?action-id) (action-name ?operator-id)
                     (param-names $?param-names) (param-values $?param-values)
                     (precondition nil))
  (domain-operator (name ?operator-id) (param-names $?op-param-names&:(= (length$ ?param-names) (length$ ?op-param-names))))
	(pddl-formula (part-of ?operator-id))
  =>
  (bind ?grounding (ground-pddl-formula ?operator-id ?param-names ?param-values nil))
  (modify ?p (precondition ?grounding))
)

(defrule domain-retract-grounding-for-plan-action-if-precondition-mismatch
  "Sometimes it is practical to switch the params of a plan-action.
  Remove grounding to trigger the grounding process again in such a case."
  ?g <- (pddl-grounding (param-values $?param-values) (id ?grounding))
  ?a <- (plan-action (precondition ?grounding) (param-values ~$?param-values))
  =>
  (retract ?g)
  (modify ?a (precondition nil))
)

(defrule domain-check-if-action-precondition-is-satisfied
  "Check if there is a referenced precondition formula that is satisfied,
  if yes make the action executable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?p <- (plan-action (executable FALSE) (id ?id) (precondition ?grounding-id)
                     (action-name ?operator-id) (state ?state))
  (pddl-formula (part-of ?operator-id) (id ?formula-id))
  (grounded-pddl-formula (is-satisfied TRUE) (formula-id ?formula-id) (grounding ?grounding-id))
  (pddl-grounding (id ?grounding-id))
  =>
  (modify ?p (executable TRUE))
  (if (eq ?state PENDING) then
    (printout t "Action " ?id " is executable based on " ?formula-id crlf)
  )
)

(defrule domain-check-if-action-precondition-is-unsatisfied
  "Check if all referenced precondition formulas are not satisfied,
  if yes make the action not executable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?p <- (plan-action (executable TRUE) (id ?id) (precondition ?grounding-id)
                     (action-name ?operator-id) (state ?state))
  (pddl-formula (part-of ?operator-id) (id ?formula-id))
  (not (grounded-pddl-formula (is-satisfied TRUE) (formula-id ?formula-id) (grounding ?grounding-id)))
  (pddl-grounding (id ?grounding-id))
  =>
  (modify ?p (executable FALSE))
  (if (eq ?state PENDING) then
    (printout t "Action " ?id " is no longer executable" crlf)
  )
)

(defrule domain-check-if-atomic-formula-is-satisfied
  "An atomic formula is satisfied when its associated predicate is satisfied."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type atom))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (is-satisfied FALSE)
                                    (grounding ?grounding-id))

  (and (pddl-predicate (part-of ?parent-base) (id ?child-base))
        (grounded-pddl-predicate (predicate-id ?child-base)
                                (grounding ?grounding-id)
                                (is-satisfied TRUE))
  )
  =>
  (modify ?parent (is-satisfied TRUE))
)

(defrule domain-check-if-atomic-formula-is-unsatisfied
  "An atomic formula is unsatisfied when its associated predicate is unsatisfied."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type atom))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (is-satisfied TRUE)
                                    (grounding ?grounding-id))

  (and (pddl-predicate (part-of ?parent-base) (id ?child-base))
        (grounded-pddl-predicate (predicate-id ?child-base)
                                (grounding ?grounding-id)
                                (is-satisfied FALSE))
  )
  =>
  (modify ?parent (is-satisfied FALSE))
)


(defrule domain-check-if-negated-formula-is-satisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type negation))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (is-satisfied FALSE)
                                    (grounding ?grounding-id))

  ; the formula is satisfied when the child is not satisifed
  (pddl-formula (part-of ?parent-base) (id ?child-base))
  (grounded-pddl-formula (formula-id ?child-base)
                         (grounding ?grounding-id)
                         (is-satisfied FALSE))
=>
  (modify ?parent (is-satisfied TRUE))
)

(defrule domain-check-if-negated-formula-is-unsatisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type negation))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (is-satisfied TRUE)
                                    (grounding ?grounding-id))

  ; the formula is unsatisfied when the child is satisifed
  (pddl-formula (part-of ?parent-base) (id ?child-base))
  (grounded-pddl-formula (formula-id ?child-base)
                         (grounding ?grounding-id)
                         (id ~nil)
                         (is-satisfied TRUE))
=>
  (modify ?parent (is-satisfied FALSE))
)

(defrule domain-check-if-conjunctive-formula-is-satisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type conjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (is-satisfied FALSE)
                                    (grounding ?grounding-id))
 ; the formula is satisfied when there is no unsatisifed child
  (not
    (and
      (pddl-formula (part-of ?parent-base) (id ?child-base))
      (grounded-pddl-formula (formula-id ?child-base)
                             (grounding ?grounding-id)
                             (is-satisfied FALSE)
      )
    )
  )
=>
  (modify ?parent (is-satisfied TRUE))
)

(defrule domain-check-if-conjunctive-formula-is-unsatisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type conjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (is-satisfied TRUE)
                                    (grounding ?grounding-id))

   ; the formula is unsatisfied when there is an unsatisifed child
  (pddl-formula (part-of ?parent-base) (id ?child-base))
  (grounded-pddl-formula (formula-id ?child-base)
                         (grounding ?grounding-id)
                         (is-satisfied FALSE)
  )
=>
  (modify ?parent (is-satisfied FALSE))
)

(defrule domain-check-if-disjunctive-formula-is-satisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))

  (pddl-formula (id ?parent-base) (type disjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (is-satisfied FALSE)
                                    (grounding ?grounding-id))

  ; the formula is satisfied when there is a satisifed child
  (pddl-formula (part-of ?parent-base) (id ?child-base))
  (grounded-pddl-formula (formula-id ?child-base)
                         (grounding ?grounding-id)
                         (is-satisfied TRUE)
  )
 =>
  (modify ?parent (is-satisfied TRUE))
)

(defrule domain-check-if-disjunctive-formula-is-unsatisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id))
  (pddl-formula (id ?parent-base) (type disjunction))
  ?parent <- (grounded-pddl-formula (id ?id)
                                    (formula-id ?parent-base)
                                    (is-satisfied TRUE)
                                    (grounding ?grounding-id))

  ; the formula is unsatisfied when there is no satisifed child
  (not
    (and
      (pddl-formula (part-of ?parent-base) (id ?child-base))
      (grounded-pddl-formula (formula-id ?child-base)
                              (grounding ?grounding-id)
                              (is-satisfied TRUE)
      )
    )
  )
=>
  (modify ?parent (is-satisfied FALSE))
)

(deffunction domain-build-ground-parameter-list
  "For multislot of parameter names, build a corresponding multislot of grounded
  parameter values and - if available - constants."
  (?names ?constants ?grounded-names ?grounded-values)

  (bind ?values (create$))
  (foreach ?param ?names
    (if (neq (nth$ ?param-index ?constants) nil) then
      (bind ?values
        (insert$ ?values ?param-index (nth$ ?param-index ?constants)))
    else
      (bind ?index (member$ ?param ?grounded-names))
      (if (not ?index) then
        (assert (domain-error (error-type unknown-parameter) (error-msg
          (str-cat "PDDL Predicate has unknown parameter " ?param)))
        )
        (return FALSE)
      else
        (bind ?values
          (insert$ ?values ?param-index (nth$ ?index ?grounded-values)))
      )
    )
  )

  (return ?values)
)

(deffunction domain-check-grounding-match
  "check if a multislot of values matches a the expected values for a multislot
  of parameter names"
  (?param-names ?domain-values ?predicate-constants ?grounded-params ?grounded-values)

  (bind ?values (domain-build-ground-parameter-list ?param-names
                                                   ?predicate-constants
                                                   ?grounded-params
                                                   ?grounded-values))

  (if (eq ?domain-values ?values) then
    (return TRUE)
  )
  (return FALSE)
)

(deffunction domain-check-grounded-equality
  (?param-names ?predicate-constants ?grounded-params ?grounded-values)

  (bind ?values (domain-build-ground-parameter-list ?param-names
                                                   ?predicate-constants
                                                   ?grounded-params
                                                   ?grounded-values))

  (if (eq (length ?values) 2) then
    (return (eq (nth$ 1 ?values) (nth$ 2 ?values)))
  )
  (return FALSE)
)

(defrule domain-check-grounded-predicate
  "A predicate is satisfied if there is a corresponding fact with a
  matching value regarding the grounding of the predicate."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (pddl-grounding (id ?grounding-id)
                  (param-names $?grounded-params)
                  (param-values $?grounded-values))

  ?base-predicate <- (pddl-predicate (id ?id)
                                     (predicate ?pred&~EQUALITY)
                                     (param-names $?param-names)
                                     (param-constants $?predicate-constants))
  ?predicate <- (grounded-pddl-predicate (predicate-id ?id)
                                         (grounding ?grounding-id)
                                         (is-satisfied FALSE))

  (domain-fact (name ?pred)
               (param-values $?domain-values&:(domain-check-grounding-match ?param-names
                                                                            ?domain-values
                                                                            ?predicate-constants
                                                                            ?grounded-params
                                                                            ?grounded-values)))
=>
  (modify ?predicate (is-satisfied TRUE))
)

(defrule domain-check-if-predicate-is-unsatisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (pddl-grounding (id ?grounding-id)
                  (param-names $?grounded-params)
                  (param-values $?grounded-values))

  ?base-predicate <- (pddl-predicate (id ?id)
                                     (predicate ?pred&~EQUALITY)
                                     (param-names $?param-names)
                                     (param-constants $?predicate-constants))
  ?predicate <- (grounded-pddl-predicate (predicate-id ?id)
                                         (grounding ?grounding-id)
                                         (is-satisfied TRUE))

  (not (domain-fact (name ?pred)
                    (param-values $?domain-values&:(domain-check-grounding-match ?param-names
                                                                                 ?domain-values
                                                                                 ?predicate-constants
                                                                                 ?grounded-params
                                                                                 ?grounded-values))))
=>
  (modify ?predicate (is-satisfied FALSE))
)


(defrule domain-check-if-predicate-equality-is-satisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id)
                  (param-names $?grounded-params)
                  (param-values $?grounded-values))
  ?predicate <- (grounded-pddl-predicate (predicate-id ?id)
                                         (grounding ?grounding-id)
                                         (is-satisfied FALSE))
  ?base-predicate <- (pddl-predicate (id ?id)
                                     (predicate EQUALITY)
                                     (param-constants $?predicate-constants)
                                     (param-names $?param-names&:
                                                  (domain-check-grounded-equality ?param-names
                                                                                  ?predicate-constants
                                                                                  ?grounded-params
                                                                                  ?grounded-values)))
=>
  (modify ?predicate (is-satisfied TRUE))
)

(defrule domain-check-if-predicate-equality-is-unsatisfied
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))

  (pddl-grounding (id ?grounding-id)
                  (param-names $?grounded-params)
                  (param-values $?grounded-values))
  ?predicate <- (grounded-pddl-predicate (predicate-id ?id)
                                         (grounding ?grounding-id)
                                         (is-satisfied TRUE))
  ?base-predicate <- (pddl-predicate (id ?id)
                                     (predicate EQUALITY)
                                     (param-constants $?predicate-constants)
                                     (param-names $?param-names&:
                                                  (not (domain-check-grounded-equality ?param-names
                                                                                  ?predicate-constants
                                                                                  ?grounded-params
                                                                                  ?grounded-values))))
=>
  (modify ?predicate (is-satisfied FALSE))
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

(deffunction remove-precondition
  "Remove a sub-formula from its parent and clean up the formula tree.
   If the parent is a disjunction with no other disjunct, simplify it to
   true by removing it recursively. If it is a negation, remove it recursively.
   If it's a conjunction, only remove the conjunct.
   If the top-most formula node is removed, replace it by a trivially true one
   (empty conjunction)."
  (?precond-fact)

  (if (not (do-for-fact ((?parent pddl-formula))
               (eq (fact-slot-value ?precond-fact part-of) ?parent:id)
    (if (or (eq (fact-slot-value ?parent type) negation)
            (eq (fact-slot-value ?parent type) atom))then
      (remove-precondition ?parent)
    else
      (if (and (eq (fact-slot-value ?parent type) disjunction)
               (not (any-factp ((?sibling pddl-formula)) (eq (fact-slot-value ?parent id)
                                                             ?sibling:part-of))
               )
               (not (any-factp ((?sibling pddl-predicate)) (eq (fact-slot-value ?parent id)
                                                               ?sibling:part-of))
               )
          )
        then
        (remove-precondition ?parent)
      )
    )
  ))
  then
      (assert (pddl-formula (id (fact-slot-value ?precond-fact id))
                            (part-of (fact-slot-value ?precond-fact part-of))
                            (type conjunction)
              )
      )
  )
  (retract ?precond-fact)
)

(deffunction domain-retract-grounding
  "Retract all groundings and grounded formulas associated with plan-actions"
  ()
  (do-for-all-facts ((?grounding pddl-grounding)) TRUE
                    (if (any-factp ((?plan-action plan-action))
                                      (eq ?plan-action:precondition ?grounding:id))
                      then
                      (do-for-all-facts ((?precond grounded-pddl-predicate))
                                        (eq ?precond:grounding ?grounding:id)
                        (retract ?precond))
                      (do-for-all-facts ((?precond grounded-pddl-formula))
                                        (eq ?precond:grounding ?grounding:id)
                        (retract ?precond))
                    )
                    (do-for-all-facts ((?plan-action plan-action))
                                      (eq ?plan-action:precondition ?grounding:id)
                      (modify ?plan-action (precondition nil))
                    )
                    (retract ?grounding)
  )
)

(deffunction domain-is-formula-negative
  "Check if a formula is negative by checking all its parents
   and counting the number of negations. If the number is odd, the formula
   is negative, otherwise it's positive."
  (?precond-id)
  (do-for-fact
    ((?precond pddl-formula))
    (eq ?precond:id ?precond-id)
    (if (any-factp ((?op domain-operator)) (eq ?op:name ?precond:part-of)) then
      return (eq ?precond:type negation)
    )
    (bind ?parent-is-negative (domain-is-formula-negative ?precond:part-of))
    (return (neq (eq ?precond:type negation) ?parent-is-negative))
  )
)

(deffunction domain-is-precond-negative
  "Mapping of the old domain-is-precond-negative to the new replacement function
  for PDDL formulas for compatibility reasons."
  (?precond-name)
  (return (domain-is-formula-negative ?precond-name))
)

(deffunction domain-get-operator-for-pddl-predicate
  (?part-of)

  (do-for-fact ((?formula pddl-formula)) (eq ?formula:id ?part-of)
    (return (domain-get-operator-for-pddl-predicate ?formula:part-of))
  )
  (do-for-fact ((?operator domain-operator)) (eq ?operator:name ?part-of)
    (return ?part-of)
  )
  (return FALSE)
)

(defrule domain-remove-precond-on-sensed-nonval-effect-of-exog-action
  "If an exogenous action has a precondition for a non-value predicate that is
  also a sensed effect of the operator, then remove the precondition on the
  effect. This means that part of the exogenous action may already have
  occurred before the action is selected."

  (domain-operator (name ?op) (exogenous TRUE))
  (domain-predicate (name ?pred) (sensed TRUE) (value-predicate FALSE))
  (domain-effect (part-of ?op)
                 (predicate ?pred)
                 (param-names $?params)
                 (param-constants $?constants))
  ?pre <- (pddl-predicate (part-of ?precond&:(eq (domain-get-operator-for-pddl-predicate ?precond)
                                                 ?op))
                          (param-names $?params)
                          (param-constants $?constants)
                          (predicate ?pred))
  =>
  (remove-precondition ?pre)
  (domain-retract-grounding)
)

(defrule domain-replace-precond-on-sensed-val-effect-of-exog-action
  (domain-operator (name ?op) (exogenous TRUE))
  (domain-predicate (name ?pred) (sensed TRUE) (value-predicate TRUE))
  (domain-effect (part-of ?op)
                 (predicate ?pred)
                 (type POSITIVE)
                 (param-names $?args ?eff-val)
                 (param-constants $?const-args ?const-eff-val))

  ?pre <- (pddl-predicate (part-of ?parent&:(eq (domain-get-operator-for-pddl-predicate ?parent)
                                                ?op))
                          (predicate ?pred)
                          (id ?precond)
                          (param-names  $?args ?cond-val)
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
                                (neq ?cond-val ?eff-val)
                            )
                          )
  )
  (not (and (pddl-formula (type disjunction) (id ?parent))
            (pddl-predicate (part-of ?parent)
                            (predicate ?pred)
                            (param-names $?args ?eff-val)
                            (param-constants $?const-args ?const-eff-val))
       )
  )
=>
  ; Replace th pddl predicate by a disjunction and add the original predicate
  ; as a disjunct. Add the effect as another disjunct.
  (assert (pddl-formula (type disjunction) (id ?precond) (part-of ?parent)))
  (assert (pddl-predicate (id (sym-cat ?precond 2)) (part-of ?precond) (predicate ?pred)
                          (param-names $?args ?eff-val) (param-constants $?const-args ?const-eff-val)))
  (modify ?pre (part-of ?precond) (id (sym-cat ?precond 1)))

  ; If there are any grounded preconditions, we need to recompute them.
  (domain-retract-grounding)
)

(defrule domain-ground-effect-precondition
  "Ground a non-atomic precondition. Grounding here merely means that we
   duplicate the precondition and tie it to one specific effect-id."
  (declare (salience ?*SALIENCE-DOMAIN-GROUND*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?pa <- (plan-action (action-name ?op)
                      (id ?action-id)
                      (goal-id ?g)
                      (plan-id ?p)
                      (state EXECUTION-SUCCEEDED)
                      (param-names $?param-names)
                      (param-values $?param-values)
                      (precondition nil))
  (domain-effect (name ?effect-name) (part-of ?op))
  ?precond <- (pddl-formula (id ?precond-id) (part-of ?effect-name))
=>
  (bind ?grounding (ground-pddl-formula ?effect-name ?param-names ?param-values nil))
  (modify ?pa (precondition ?grounding))
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

(defrule domain-effects-check-for-sensed
  "Apply effects of an action after it succeeded."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?pa <- (plan-action (id ?id) (goal-id ?g) (plan-id ?p) (action-name ?op)
                      (state EXECUTION-SUCCEEDED)
                      (param-names $?action-param-names)
                      (param-values $?action-param-values)
                      (precondition ?grounding-id))
  (domain-operator (name ?op))
  =>
  (bind ?next-state SENSED-EFFECTS-HOLD)
  (do-for-all-facts ((?e domain-effect) (?pred domain-predicate))
    (and ?pred:sensed (eq ?e:part-of ?op) (eq ?e:predicate ?pred:name))
    ; apply if this effect is unconditional or the condition is satisfied
    (if (or (not (any-factp ((?cep pddl-formula)) (eq ?cep:part-of ?e:name)))
            (any-factp ((?cep grounded-pddl-formula))
                       (and (eq ?cep:part-of ?e:name) ?cep:is-satisfied
                            (any-factp ((?grounding pddl-grounding))
                                       (and (eq ?cep:grounding ?grounding:id)
                                            (eq ?grounding:id ?grounding-id)
                                       )
                             )
                       )
            )
        )
    then
      (bind ?values
            (domain-ground-effect ?e:param-names ?e:param-constants
                                  ?action-param-names ?action-param-values))

      (assert (domain-pending-sensed-fact (name ?pred:name) (action-id ?id) (goal-id ?g) (plan-id ?p)
                                          (param-values ?values) (type ?e:type)))
      (bind ?next-state SENSED-EFFECTS-WAIT)
    )
  )
  (modify ?pa (state ?next-state))
)

(defrule domain-effects-ignore-sensed
  "Do not wait for sensed effects if the operator is not a waiting operator."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?pa <- (plan-action	(id ?id) (action-name ?op) (state SENSED-EFFECTS-WAIT))
	(domain-operator (name ?op) (wait-sensed FALSE))
	=>
	(modify ?pa (state SENSED-EFFECTS-HOLD))
)

(defrule domain-effects-apply
  "Apply effects of an action after it succeeded."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  (goal (id ?g))
  (plan (id ?p) (goal-id ?g))
  ?pa <- (plan-action	(id ?id) (goal-id ?g) (plan-id ?p) (action-name ?op)
                      (state SENSED-EFFECTS-HOLD)
                      (param-names $?action-param-names)
                      (param-values $?action-param-values)
                      (precondition ?grounding-id))
  (domain-operator (name ?op))
  =>
  (do-for-all-facts ((?e domain-effect) (?pred domain-predicate))
    (and (not ?pred:sensed) (eq ?e:part-of ?op) (eq ?e:predicate ?pred:name))

    ; apply if this effect is unconditional or the condition is satisfied
    (if (or (not (any-factp ((?cep pddl-formula)) (eq ?cep:part-of ?e:name)))
            (any-factp ((?cep grounded-pddl-formula))
                       (and (eq ?cep:part-of ?e:name) ?cep:is-satisfied
                            (any-factp ((?grounding pddl-grounding))
                                       (and (eq ?cep:grounding ?grounding:id)
                                            (eq ?grounding:id ?grounding-id)
                                       )
                             )
                       )
            )
        )
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
  (modify ?pa (state EFFECTS-APPLIED))
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
  ?a <- (plan-action (id ?action-id) (state SENSED-EFFECTS-WAIT) (plan-id ?p) (goal-id ?g))
  (not (domain-pending-sensed-fact (action-id ?action-id) (goal-id ?g) (plan-id ?p)))
  =>
  (modify ?a (state SENSED-EFFECTS-HOLD))
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
  ?a <- (plan-action (id ?action-id) (state EFFECTS-APPLIED))
  =>
  (modify ?a (state FINAL))
  (domain-retract-grounding)
)

(defrule domain-action-failed
  "An action has failed."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?a <- (plan-action (id ?action-id) (state EXECUTION-FAILED))
  =>
  (modify ?a (state FAILED))
  (domain-retract-grounding)
)

(defrule domain-check-if-action-is-executable-without-precondition
  "If the precondition of an action does not exist, the action is alwaysexecutable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?action <- (plan-action (id ?action-id)
                          (action-name ?action-name)
                          (executable FALSE)
                          (precondition nil))
=>
  (modify ?action (executable TRUE))
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

(defrule domain-check-pddl-predicate-has-domain-prediacte
  "Make sure that all preconditions have a predicate or are set to equality."
  (pddl-predicate
    (id ?precond)
    (predicate nil)
  )
=>
  (assert (domain-error
    (error-type precondition-must-have-predicate-or-be-equality)
    (error-msg (str-cat "pddl-predicate " ?precond " must have a predicate"))))
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


(defrule domain-check-grounded-pddl-equality-has-two-params
  (pddl-grounding (id ?grounding-id)
                  (param-names $?grounding-names)
                  (param-values $?grounding-values))
  (grounded-pddl-predicate (grounding ?grounding-id)
                           (predicate-id ?pid)
                           (id ?id))
  (pddl-predicate (id ?pid)
                  (predicate EQUALITY)
                  (param-names $?param-names)
                  (param-constants $?param-constants&:
                    (not (= (length$ (domain-build-ground-parameter-list ?param-names
                                                                         ?param-constants
                                                                         ?grounding-names
                                                                         ?grounding-values))
                                      2)))
  )
  =>
  (assert (domain-error
    (error-type equality-must-have-exactly-two-parameters)
    (error-msg (str-cat "Predicate " ?id " is an equality precondition"
                        " but has " (length$ ?param-names) " parameters,"
                        " should be 2."))))
)
