;---------------------------------------------------------------------------
;  test-domain.clp - Test Domain
;
;  Created: Wed 04 Oct 2017 18:14:57 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defrule load-domain
  (executive-init)
  (not (domain-loaded))
=>
  (assert
    (domain-loaded)
    (domain-object-type (name text))

    (domain-predicate (name said) (param-names c))

    (domain-operator (name say-hello))
    (domain-precondition (part-of say-hello) (type conjunction))
    (domain-effect
      (part-of say-hello) (predicate said) (param-names hello))

    (domain-operator (name say-goodbye))
    (domain-precondition
      (name say-goodbye-precond) (part-of say-goodbye) (type conjunction))
    (domain-atomic-precondition
      (part-of say-goodbye-precond)
      (predicate said)
      (param-names c)
      (param-constants hello)
    )
    (domain-effect
      (part-of say-goodbye) (predicate said) (param-names goodbye))
    (domain-effect
      (part-of say-goodbye) (predicate said) (param-names hello) (type NEGATIVE))
  )
)
