(deffacts blocksworld
  "A simple blocksworld domain"
  (domain-object-type (name block) (super-type object))
  ; pick-up
  (domain-operator (name pick-up))
  (domain-operator-parameter (operator pick-up) (type block) (name x))
  (domain-precondition
    (part-of pick-up) (name pick-up-precond) (type conjunction))
  (domain-precondition
    (part-of pick-up-precond) (name neg-on-table) (type negation))
  (domain-atomic-precondition
    (part-of neg-on-table) (param-names (create$ x)) (predicate ontable))
  (domain-atomic-precondition
    (part-of pick-up-precond) (param-names (create$ x)) (predicate clear))
  (domain-atomic-precondition
    (part-of pick-up-precond) (predicate handempty))
  (domain-effect
    (part-of pick-up) (predicate ontable) (type NEGATIVE)
    (param-names (create$ x)))
  (domain-effect
    (part-of pick-up) (predicate clear) (type NEGATIVE)
    (param-names (create$ x)))
  (domain-effect
    (part-of pick-up) (predicate handempty) (type NEGATIVE))
  (domain-effect
    (part-of pick-up) (predicate holding) (param-names (create$ x)))
  ; unstack
  (domain-operator (name unstack))
  (domain-operator-parameter (operator unstack) (type block) (name x))
  (domain-operator-parameter (operator unstack) (type block) (name y))
  (domain-precondition
    (part-of unstack) (name unstack-precond) (type conjunction))
  (domain-atomic-precondition
    (part-of unstack-precond) (param-names (create$ x y)) (predicate on))
  (domain-atomic-precondition
    (part-of unstack-precond) (param-names (create$ x)) (predicate clear))
  (domain-atomic-precondition
    (part-of unstack-precond) (predicate handempty))
  (domain-effect
    (part-of unstack) (predicate holding) (param-names (create$ x)))
  (domain-effect
    (part-of unstack) (predicate clear) (param-names (create$ y)))
  (domain-effect
    (part-of unstack) (predicate clear)  (type NEGATIVE)
    (param-names (create$ x)))
  (domain-effect
    (part-of unstack) (predicate on) (type NEGATIVE)
    (param-names (create$ x y)))

  ; world model
  (domain-fact (name handempty))
  (domain-fact (name clear) (param-values b1))
  (plan-action (id 1) (plan-id p0) (action-name pick-up) (param-names (create$ x))
    (param-values (create$ b1)))

  (domain-fact (name on) (param-values b1 b2))
  (plan-action (id 2) (plan-id p0) (action-name unstack)
    (param-names (create$ x y)) (param-values (create$ b1 b2)))
)

(defrule apply-action
  "Pseudo-execute action by changing its state to EXECUTED."
  ?aa <- (apply-action ?action-id)
  ?pa <- (plan-action (id ?action-id))
=>
  (modify ?pa (status EXECUTED))
  (retract ?aa)
)
