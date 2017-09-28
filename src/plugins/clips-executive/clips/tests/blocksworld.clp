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
    (part-of neg-on-table) (parameters (create$ x)) (predicate ontable))
  (domain-atomic-precondition
    (part-of pick-up-precond) (parameters (create$ x)) (predicate clear))
  (domain-atomic-precondition
    (part-of pick-up-precond) (predicate handempty))
  ; unstack
  (domain-operator (name unstack))
  (domain-operator-parameter (operator unstack) (type block) (name x))
  (domain-operator-parameter (operator unstack) (type block) (name y))
  (domain-precondition
    (part-of unstack) (name unstack-precond) (type conjunction))
  (domain-atomic-precondition
    (part-of unstack-precond) (parameters (create$ x y)) (predicate on))
  (domain-atomic-precondition
    (part-of unstack-precond) (parameters (create$ x)) (predicate clear))
  (domain-atomic-precondition
    (part-of unstack-precond) (predicate handempty))

  ; world model
  (domain-predicate (name handempty))
  (domain-predicate (name clear) (parameters b1))
  (domain-grounding (parameter x) (value b1) (operator pick-up))

  (domain-predicate (name on) (parameters b1 b2))
  (domain-grounding (parameter x) (value b1) (operator unstack))
  (domain-grounding (parameter y) (value b2) (operator unstack))
)
