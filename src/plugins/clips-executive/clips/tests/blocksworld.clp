(deffacts blocksworld
  "A simple blocksworld domain"
  (obj-type (name obj) (super-type nil))
  (obj-type (name block) (super-type obj))
  ; pick-up
  (operator (name pick-up))
  (parameter (operator pick-up) (obj-type block) (name x))
  (precondition (part-of pick-up) (name pick-up-precond) (type conjunction))
  (precondition (part-of pick-up-precond) (name neg-on-table) (type negation))
  (atomic-precondition
    (part-of neg-on-table) (parameters (create$ x)) (predicate ontable))
  (atomic-precondition
    (part-of pick-up-precond) (parameters (create$ x)) (predicate clear))
  (atomic-precondition
    (part-of pick-up-precond) (predicate handempty))
  ; unstack
  (operator (name unstack))
  (parameter (operator unstack) (obj-type block) (name x))
  (parameter (operator unstack) (obj-type block) (name y))
  (precondition (part-of unstack) (name unstack-precond) (type conjunction))
  (atomic-precondition
    (part-of unstack-precond) (parameters (create$ x y)) (predicate on))
  (atomic-precondition
    (part-of unstack-precond) (parameters (create$ x)) (predicate clear))
  (atomic-precondition
    (part-of unstack-precond) (predicate handempty))

  ; world model
  (predicate (name handempty))
  (predicate (name clear) (parameters b1))
  (grounding (parameter x) (value b1) (operator pick-up))

  (predicate (name on) (parameters b1 b2))
  (grounding (parameter x) (value b1) (operator unstack))
  (grounding (parameter y) (value b2) (operator unstack))
)
