;---------------------------------------------------------------------------
;  fixed-sequence.clp - Goal expander for promise-test goals
;
;  Created: Thu Dec 2 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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


(defrule goal-expander-get-container-and-fill
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class FILL-CONTAINER) (mode SELECTED)
              (params robot ?r container ?c start ?start mine ?mine) (parent ?parent-id))
  =>
  (bind ?plan-id (sym-cat FILL-CONTAINER-PLAN (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name move-plaza) (param-values ?r ?start CONTAINER-DEPOT) )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name pick-container) (param-values ?r ?c))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name move) (param-values ?r CONTAINER-DEPOT ?mine))
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name collect-regolith) (param-values ?r ?mine ?c))
  )
  (modify ?g (mode EXPANDED) (committed-to ?plan-id))
)


(defrule goal-expander-deliver-to-machine
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class DELIVER) (mode SELECTED)
              (params robot ?r side ?side machine ?machine container ?c material ?mat) (parent ?parent-id))
  (domain-fact (name robot-at) (param-values ?r ?start))
  =>
  (bind ?plan-id (sym-cat DELIVER-PLAN (gensym*)))
  (if (eq ?mat REGOLITH) then
    (assert
      (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
      (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move) (param-values ?r ?start ?side))
      (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name put-regolith) (param-values ?r ?side ?machine ?c))
      (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move-plaza) (param-values ?r ?side BASE))
    )
    else
    (assert
      (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
      (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move) (param-values ?r ?start ?side))
      (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name put-processite) (param-values ?r ?side ?machine ?c))
      (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move-plaza) (param-values ?r ?side BASE))
    )
  )
  (modify ?g (mode EXPANDED) (committed-to ?plan-id))
)


(defrule goal-expander-start-machine
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class START-MACHINE) (mode SELECTED)
              (params robot ?r side ?s machine ?m) (parent ?parent-id))
  =>
  (bind ?plan-id (sym-cat START-MACHINE-PLAN (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name start-machine) (param-values ?m))
  )
  (modify ?g (mode EXPANDED) (committed-to ?plan-id))
)


(defrule goal-expander-clean-machine
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class CLEAN-MACHINE) (mode SELECTED)
              (params robot ?r side ?s machine ?m container ?c material ?mat) (parent ?parent-id))
  (domain-fact (name robot-at) (param-values ?r ?start))
  =>
  (bind ?plan-id (sym-cat CLEAN-MACHINE-PLAN (gensym*)))
  (if (eq ?mat PROCESSITE) then
    (assert
      (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
      (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move) (param-values ?r ?start ?s))
      (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name collect-processite) (param-values ?r ?m ?s ?c))
      (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move-plaza) (param-values ?r ?s BASE))
    )
    else
    (assert
      (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
      (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move) (param-values ?r ?start ?s))
      (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name collect-xenonite) (param-values ?r ?m ?s ?c))
      (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move-plaza) (param-values ?r ?s BASE))
    )
  )
  (modify ?g (mode EXPANDED) (committed-to ?plan-id))
)

(defrule goal-expander-deliver-xenonite
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class DELIVER-XENONITE) (mode SELECTED)
              (params robot ?r container ?c) (parent ?parent-id))
  (domain-fact (name robot-at) (param-values ?r ?start))
  =>
  (bind ?plan-id (sym-cat DELIVER-XENONITE-PLAN (gensym*)))

  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                (action-name move) (param-values ?r ?start STORAGE-INPUT))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                (action-name put-container) (param-values ?r ?c STORAGE-INPUT))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                (action-name move-plaza) (param-values ?r STORAGE-INPUT BASE))
  )

  (modify ?g (mode EXPANDED) (committed-to ?plan-id))
)
