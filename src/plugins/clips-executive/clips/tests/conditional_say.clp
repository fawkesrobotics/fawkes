;---------------------------------------------------------------------------
;  conditional_say.clp - A test domain with a conditional effect
;
;  Created: Fri 15 Dec 2017 14:12:39 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;

(deffacts say
  (domain-object-type (name text))
  (domain-object-type (name speaker))
	(domain-predicate (name said) (param-names t))
	(domain-predicate (name speaker-ready) (param-names s))
  (domain-operator (name say))
  (domain-operator-parameter (operator say) (type text) (name t))
  (domain-operator-parameter (operator say) (type speaker) (name s))
  (domain-precondition (part-of say))
  (domain-effect
    (name say-effect) (part-of say) (predicate said) (param-names t))
  (domain-precondition (part-of say-effect) (name say-eff-cond))
  (domain-atomic-precondition
    (part-of say-eff-cond) (predicate speaker-ready) (param-names s))
)

(defrule apply-action
  "Pseudo-execute action by changing its state to FINAL."
  ?aa <- (apply-action ?goal-id ?plan-id ?action-id)
  ?pa <- (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id))
=>
  (modify ?pa (status EXECUTION-SUCCEEDED))
  (retract ?aa)
)
