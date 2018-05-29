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
  (parse-pddl-domain (path-resolve "test-scenario/domain.pddl"))
  (assert (domain-loaded))
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
  (assert
	 (domain-object (name hello) (type text))
	 (domain-object (name goodbye) (type text))
	 (domain-object (name lock1) (type name))
	 (domain-operator (name print) (param-names severity text))
	 (domain-operator (name say-cleanup))
	 (domain-fact (name said) (param-values bob hello))
	 (domain-facts-loaded)
	)
)

(defrule test-domain-set-domain-fact-said-hello
  (plan-action (action-name say-hello|say-hello-again) (param-values peggy) (status SENSED-EFFECTS-WAIT))
=>
  (assert (domain-fact (name said) (param-values peggy hello)))
)

(defrule test-domain-set-domain-fact-said-goodbye
  (plan-action (action-name say-goodbye) (status SENSED-EFFECTS-WAIT))
=>
  (assert (domain-fact (name said) (param-values peggy goodbye)))
)
