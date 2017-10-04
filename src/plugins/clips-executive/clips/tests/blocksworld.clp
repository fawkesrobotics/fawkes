(deffacts blocksworld
  "A simple blocksworld domain"
  (domain-object-type (name obj) (super-type nil))
  (domain-object-type (name block) (super-type obj))
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
;  (domain-effect
;    (part-of unstack) (predicate holding) (parameters (create$ x)))
;  (domain-effect
;    (part-of unstack) (predicate clear) (parameters (create$ y)))
;  (domain-effect
;    (part-of unstack) (predicate clear)  (type NEGATIVE)
;    (parameters (create$ x)))
;  (domain-effect
;    (part-of unstack) (predicate on) (type NEGATIVE)
;    (parameters (create$ x y)))

  ; world model
  (domain-predicate (name handempty))
  (domain-predicate (name clear) (parameters b1))
  (plan-action (id 1) (plan-id p0) (action-name pick-up) (param-names (create$ x))
    (param-values (create$ b1)))

  (domain-predicate (name on) (parameters b1 b2))
  (plan-action (id 2) (plan-id p0) (action-name unstack)
    (param-names (create$ x y)) (param-values (create$ b1 b2)))
)
