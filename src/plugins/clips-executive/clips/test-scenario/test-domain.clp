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

(defrule load-initial-facts
  (executive-init)
  =>
  (assert (domain-fact (name said) (param-values bob hello)))
)
