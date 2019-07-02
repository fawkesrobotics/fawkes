;---------------------------------------------------------------------------
;  resource-locks.clp - Resource locking for goal resources
;
;  Created: Thu 31 May 2018 16:51:33 CEST
;  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

(deftemplate resource-request
	(slot resource (type SYMBOL))
	(slot goal (type SYMBOL))
)

(deffunction resource-to-mutex (?resource)
  "Get the name of the mutex for the given resource."
  (return (sym-cat resource- ?resource))
)

(deffunction mutex-to-resource (?mutex)
  "Get the resource name of a resource mutex."
  (if (str-prefix resource- ?mutex)  then
    (return
      (sym-cat (sub-string (+ (length$ resource-) 1) (length$ ?mutex) ?mutex)))
  )
  (return FALSE)
)

(defrule resource-locks-request-lock
  "Request a mutex for each required resource if none of the respective mutexes
   is already locked or has a pending request."
  (goal (mode COMMITTED)
        (id ?goal-id)
        (verbosity ?verbosity)
        (acquired-resources)
        (required-resources $?req&:(> (length$ ?req) 0)))
  (not (mutex (name ?n&:(member$ (mutex-to-resource ?n) ?req))
              (request ~NONE)))
  (not (mutex (name ?n&:(member$ (mutex-to-resource ?n) ?req))
              (state LOCKED)))
  (not (resource-request (resource ?res&:(member$ ?res ?req))))
  =>
  (foreach ?res ?req
    (if (neq ?verbosity QUIET) then
      (printout warn "Locking resource " ?res crlf)
    )
    (mutex-try-lock-async (resource-to-mutex ?res))
		(assert (resource-request (goal ?goal-id) (resource ?res)))
  )
)

(defrule resource-locks-fast-reject-goal-locked-by-other-agent
  "If a resource already locked by someone else, we have not acquired any
   resources, and we have no pending requests, then we can directly reject the
   goal."
  (wm-fact (key cx identity) (value ?identity))
  ?g <- (goal (mode COMMITTED)
              (id ?goal-id)
              (verbosity ?verbosity)
              (acquired-resources)
              (required-resources $?req))
  ; The mutex is locked and there is no unprocessed request, which means that
  ; it was either locked by someone else or for a different goal.
  (mutex (name ?n&:(member$ (mutex-to-resource ?n) ?req))
         (state LOCKED) (request NONE) (locked-by ?locker&~?identity))
  (not (mutex (name ?n1&:(member$ (mutex-to-resource ?n1) ?req))
              (request ~NONE)))
  =>
  (if (neq ?verbosity QUIET) then
    (printout warn "Rejecting goal " ?goal-id ", " (mutex-to-resource ?n)
                   " is already locked by " ?locker crlf)
  )
  (modify ?g (mode FINISHED) (outcome REJECTED))
)

(defrule resource-locks-fast-reject-goal-locked-for-another-goal
  "If a resource already locked for another goal, we have not acquired any
   resources, and we have no pending requests, then we can directly reject the
   goal."
  ?g <- (goal (mode COMMITTED)
              (id ?goal)
              (verbosity ?verbosity)
              (acquired-resources)
              (required-resources $? ?res $?))
  (resource-request (resource ?res) (goal ?other-goal&~?goal))
  (not (mutex (name ?n1&:(eq ?n1 (resource-to-mutex ?res)))
              (request ~NONE)))
  =>
  (if (neq ?verbosity QUIET) then
    (printout warn "Rejecting goal " ?goal ", " ?res
                  " is already locked for " ?other-goal crlf)
  )
  (modify ?g (mode FINISHED) (outcome REJECTED))
)

(defrule resource-locks-lock-acquired
	(resource-request (resource ?res) (goal ?goal-id))
  ?m <- (mutex (name ?n&:(eq ?n (resource-to-mutex ?res)))
               (request LOCK) (response ACQUIRED))
  ?g <- (goal (mode COMMITTED) (id ?goal-id)
              (required-resources $?req)
              (acquired-resources $?acq
                &:(member$ ?res (set-diff ?req ?acq))))
  =>
  (modify ?g (acquired-resources (append$ ?acq (mutex-to-resource ?n))))
  (modify ?m (request NONE) (response NONE))
)

(defrule resource-locks-lock-rejected-release-acquired-resources
  "A lock was rejected, therefore release all acquired resources."
  ?m <- (mutex (name ?n)
               (request LOCK)
               (response REJECTED|ERROR)
               (error-msg ?err))
  ?g <- (goal (mode COMMITTED) (id ?goal-id)
              (required-resources $?req)
              (acquired-resources $?acq
                &:(member$ (mutex-to-resource ?n) (set-diff ?req ?acq))))
  ; We cannot abort a pending request. Thus, we first need to wait to get
  ; responses for all requested locks.
  (not (mutex (name ?on&:(member$ (mutex-to-resource ?on) ?req))
              (response PENDING)))
  =>
  (delayed-do-for-all-facts
    ((?om mutex))
    (and (eq ?om:request NONE) (member$ (mutex-to-resource ?om:name) ?acq))
    (mutex-unlock-async ?om:name)
  )
)

(defrule resource-locks-reject-goal-on-rejected-lock
  "A lock was rejected and no resource is acquired anymore. Reject the goal."
  (mutex (name ?res) (request LOCK) (response REJECTED|ERROR) (error-msg ?err))
  ?g <- (goal (mode COMMITTED)
              (id ?goal-id)
              (verbosity ?verbosity)
              (required-resources $?req
                &:(member$ (mutex-to-resource ?res) ?req))
              (acquired-resources))
  ; We cannot abort a pending request. Thus, we first need to wait to get
  ; responses for all requested locks.
  ; We also need to release all acquired mutexes before rejecting the goal.
  (not (mutex (name ?ores&:(member$ (mutex-to-resource ?ores) ?req))
              (response PENDING|ACQUIRED)))
  =>
  (if (neq ?verbosity QUIET) then
    (printout warn "Rejecting goal " ?goal-id ", resource lock "
                   (mutex-to-resource ?res) " was rejected" crlf)
  )
  (modify ?g (mode FINISHED) (outcome REJECTED) (message ?err))
  (delayed-do-for-all-facts
    ((?om mutex) (?request resource-request))
    (and (or (eq ?om:response REJECTED) (eq ?om:response ERROR))
         (eq ?request:resource (mutex-to-resource ?om:name)))
    (modify ?om (request NONE) (response NONE) (error-msg ""))
		(retract ?request)
  )
)

(defrule resource-locks-unlock-start
  ?g <- (goal (mode RETRACTED) (verbosity ?verbosity) (acquired-resources $?acq))
  =>
  (foreach ?res ?acq
    (if (not (any-factp ((?m mutex))
                        (and (eq ?m:name (resource-to-mutex ?res))
                             (or (eq ?m:request UNLOCK)
                                 (member$ UNLOCK ?m:pending-requests)))))
     then
      (mutex-unlock-async (resource-to-mutex ?res))
      (if (neq ?verbosity QUIET) then
        (printout warn "Unlocking resource " ?res crlf)
      )
    )
  )
)

(defrule resource-locks-unlock-done
  ?request <- (resource-request (resource ?res) (goal ?goal-id))
  ?m <- (mutex (name ?n&:(eq ?n (resource-to-mutex ?res))) (state OPEN) (request UNLOCK))
  ?g <- (goal (id ?goal-id) (acquired-resources $?acq))
  =>
  (modify ?g (acquired-resources
              (delete-member$ ?acq ?res)))
  (modify ?m (request NONE) (response NONE))
  (retract ?request)
)
