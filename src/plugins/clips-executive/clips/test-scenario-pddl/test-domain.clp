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
  (parse-pddl-domain (path-resolve "test-scenario-pddl/domain.pddl"))
  (assert (domain-loaded))
  (assert (skiller-control))
)

(defrule test-domain-set-sensed-predicates
  (executive-init)
  (domain-loaded)
  ?p <- (domain-predicate (name said) (sensed FALSE))
=>
  (modify ?p (sensed TRUE))
)

(defrule load-initial-facts
  (executive-init)
  (domain-loaded)
  =>
  (assert (domain-fact (name said) (param-values BOB HELLO)))
  (assert
          (domain-object (name HELLO) (type text))
          (domain-object (name GOODBYE) (type text))
          (domain-object (name MEH) (type text))
          (domain-object (name BOB) (type name))
          (domain-object (name PEGGY) (type name))
  )
	(pddl-robmem-flush)
  (assert (domain-facts-loaded))
)

(defrule test-domain-set-domain-fact-said-hello
  (plan-action (action-name say-hello) (param-values PEGGY) (state SENSED-EFFECTS-WAIT))
=>
  (assert (domain-fact (name said) (param-values PEGGY HELLO)))
)

(defrule test-domain-set-domain-fact-said-goodbye
  (plan-action (action-name say-goodbye) (param-values PEGGY GOODBYE) (state SENSED-EFFECTS-WAIT))
=>
  (assert (domain-fact (name said) (param-values PEGGY GOODBYE)))
)
