(deffacts blocksworld
  "A simple blocksworld domain"
  (domain-object-type (name block) (super-type object))
  ; pick-up
	(domain-predicate (name ontable) (param-names x))
	(domain-predicate (name clear) (param-names x))
	(domain-predicate (name handempty))
	(domain-predicate (name holding) (param-names x))
	(domain-predicate (name on) (param-names x y))
  (domain-operator (name pick-up))
  (domain-operator-parameter (operator pick-up) (type block) (name x))
  (domain-precondition
    (part-of pick-up) (name pick-up-precond) (type conjunction))
  (domain-precondition
    (part-of pick-up-precond) (name neg-on-table) (type negation))
  (domain-atomic-precondition
    (part-of neg-on-table) (param-names x) (predicate ontable))
  (domain-atomic-precondition
    (part-of pick-up-precond) (param-names x) (predicate clear))
  (domain-atomic-precondition
    (part-of pick-up-precond) (predicate handempty))
  (domain-effect
    (part-of pick-up) (predicate ontable) (type NEGATIVE)
    (param-names x))
  (domain-effect
    (part-of pick-up) (predicate clear) (type NEGATIVE)
    (param-names x))
  (domain-effect
    (part-of pick-up) (predicate handempty) (type NEGATIVE))
  (domain-effect
    (part-of pick-up) (predicate holding) (param-names x))
  ; unstack
  (domain-operator (name unstack))
  (domain-operator-parameter (operator unstack) (type block) (name x))
  (domain-operator-parameter (operator unstack) (type block) (name y))
  (domain-precondition
    (part-of unstack) (name unstack-precond) (type conjunction))
  (domain-atomic-precondition
    (part-of unstack-precond) (param-names x y) (predicate on))
  (domain-atomic-precondition
    (part-of unstack-precond) (param-names x) (predicate clear))
  (domain-atomic-precondition
    (part-of unstack-precond) (predicate handempty))
  (domain-effect
    (part-of unstack) (predicate holding) (param-names x))
  (domain-effect
    (part-of unstack) (predicate clear) (param-names y))
  (domain-effect
    (part-of unstack) (predicate clear)  (type NEGATIVE)
    (param-names x))
  (domain-effect
    (part-of unstack) (predicate on) (type NEGATIVE)
    (param-names x y))
  ; stack
  (domain-operator (name stack))
  (domain-operator-parameter (operator stack) (type block) (name x))
  (domain-operator-parameter (operator stack) (type block) (name y))
  (domain-precondition
    (part-of stack) (name stack-precond) (type conjunction))
  (domain-atomic-precondition
    (part-of stack-precond) (param-names x) (predicate holding))
  (domain-atomic-precondition
    (part-of stack-precond) (param-names y) (predicate clear))
  (domain-effect
    (part-of stack) (predicate holding) (param-names x) (type NEGATIVE))
  (domain-effect
    (part-of stack) (predicate clear) (param-names y) (type NEGATIVE))
  (domain-effect
    (part-of stack) (predicate clear) (param-names x))
  (domain-effect
    (part-of stack) (predicate handempty))
  (domain-effect
    (part-of stack) (predicate on) (param-names x y))

  ; world model
  (domain-fact (name handempty))
  (domain-fact (name clear) (param-values b1))
  (plan-action (id 1) (goal-id g0) (plan-id p0) (action-name pick-up) (param-names x)
    (param-values b1))

  (domain-fact (name on) (param-values b1 b2))
  (plan-action (id 2) (goal-id g0) (plan-id p0) (action-name unstack)
    (param-names x y) (param-values b1 b2))
)

(defrule apply-action
  "Pseudo-execute action by changing its state to EXECUTION-SUCCEEDED ."
  ?aa <- (apply-action ?goal-id ?plan-id ?action-id)
  ?pa <- (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id))
=>
  (modify ?pa (status EXECUTION-SUCCEEDED))
  (retract ?aa)
)
