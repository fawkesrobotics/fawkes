;---------------------------------------------------------------------------
;  noop-actions.clp - Pseudo actions that are not actually executed
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

(defglobal
  ?*MIN-EXEC-DURATION* = 4
  ?*EXEC-DURATION* = 2
  ?*MOVE-EXEC-DURATION* = 10
  ?*MACHINE-EXEC-DURATION* = 15
)

;A timeout for an action
(deftemplate action-timer
  (slot plan-id (type SYMBOL))
  (slot action-id(type NUMBER))
  (slot duration (type NUMBER))
  (multislot start-time)
)

(deftemplate machine-timer
  (slot machine (type SYMBOL))
  (slot duration (type NUMBER))
  (multislot start-time)
)

(defrule action-timer-noop-run-move
  ?p <- (plan-action (plan-id ?plan-id) (id ?id) (goal-id ?goal-id) (action-name ?op&move|move-plaza) (state PENDING) (param-values ?r ?start ?end))
  (not (plan-action (action-name move) (plan-id ~?plan-id) (goal-id ~?goal-id) (param-values ? ? ?end) (state PENDING)))
  (not (action-timer (plan-id ?plan-id) (action-id ?id)))
  (time ?now ?mills)
  =>
  (printout t "Starting action " ?op "(" ?id ") of plan " ?plan-id crlf)
  (assert
    (action-timer (plan-id ?plan-id) (action-id ?id) (start-time ?now) (duration ?*MOVE-EXEC-DURATION*))
  )
  (modify ?p (state RUNNING))
)

(defrule action-timer-noop-run
  ?p <- (plan-action (plan-id ?plan-id) (id ?id) (goal-id ?goal-id) (action-name ?op&~move) (state PENDING))
  (not (action-timer (plan-id ?plan-id) (action-id ?id)))
  (time ?now ?mills)
  =>
  (printout t "Starting action " ?op "(" ?id ") of plan " ?plan-id crlf)
  (assert
    (action-timer (plan-id ?plan-id) (action-id ?id) (start-time ?now) (duration ?*EXEC-DURATION*))
  )
  (modify ?p (state RUNNING))
)

(defrule action-timer-noop-stop
  ?p <- (plan-action (plan-id ?plan-id) (id ?id) (goal-id ?goal-id) (action-name ?op) (state RUNNING))
  ?a <- (action-timer (plan-id ?plan-id) (action-id ?id) (start-time ?time) (duration ?duration))
  (time ?now ?mills)
  (test (> ?now (+ ?duration ?time)))
  =>
  (printout t "Finished action " ?op "(" ?id ") of plan " ?plan-id crlf)
  (retract ?a)
  (modify ?p (state EXECUTION-SUCCEEDED))
)

(defrule action-start-machine-operation
  (domain-fact (name machine-in-state) (param-values ?machine OPERATING))
  (not (machine-timer (machine ?machine)))
  (time ?now ?mills)
  =>
  (printout t "Starting machine operation of " ?machine crlf)
  (assert
    (machine-timer (machine ?machine) (start-time ?now) (duration ?*MACHINE-EXEC-DURATION*))
  )
)

(defrule action-stop-machine-operation
  ?df <- (domain-fact (name machine-in-state) (param-values ?machine OPERATING))
  ?mt <- (machine-timer (machine ?machine) (start-time ?time) (duration ?duration))
  (time ?now ?mills)
  (test (> ?now (+ ?duration ?time)))
  =>
  (printout t "Stopping machine operation of " ?machine crlf)
  (retract ?mt)
  (modify ?df (param-values ?machine READY))
)