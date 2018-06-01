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
  (goal (mode COMMITTED)
        (acquired-resources $?acq)
        (required-resources $?req))
  (not (mutex (name ?n&:(member$ (mutex-to-resource ?n) ?req))
              (request ~NONE)))
  =>
  (foreach ?res (set-diff ?req ?acq)
    (printout warn "Locking resource " ?res crlf)
    (mutex-try-lock-async (resource-to-mutex ?res))
  )
)

(defrule resource-locks-lock-acquired
  ?m <- (mutex (name ?res) (request LOCK) (response ACQUIRED))
  ?g <- (goal (mode COMMITTED)
              (required-resources $?req)
              (acquired-resources $?acq
                &:(member$ (mutex-to-resource ?res) (set-diff ?req ?acq))))
  =>
  (modify ?g (acquired-resources (append$ ?acq (mutex-to-resource ?res))))
  (modify ?m (request NONE) (response NONE))
)

(defrule resource-locks-lock-rejected-release-acquired-resources
  "A lock was rejected, therefore release all acquired resources."
  ?m <- (mutex (name ?res)
               (request LOCK)
               (response REJECTED|ERROR)
               (error-msg ?err))
  ?g <- (goal (mode COMMITTED)
              (required-resources $?req)
              (acquired-resources $?acq
                &:(member$ (mutex-to-resource ?res) (set-diff ?req ?acq))))
  ; We cannot abort a pending request. Thus, we first need to wait to get
  ; responses for all requested locks.
  (not (mutex (name ?on&:(member$ (mutex-to-resource ?on) ?req))
              (response PENDING)))
  =>
  (do-for-all-facts ((?om mutex)) (member$ (mutex-to-resource ?om:name) ?acq)
    (mutex-unlock-async ?om:name)
  )
)

(defrule resource-locks-reject-goal-on-rejected-lock
  "A lock was rejected and no resource is acquired anymore. Reject the goal."
  (mutex (name ?res) (request LOCK) (response REJECTED|ERROR) (error-msg ?err))
  ?g <- (goal (mode COMMITTED)
              (required-resources $?req
                &:(member$ (mutex-to-resource ?res) ?req))
              (acquired-resources))
  ; We cannot abort a pending request. Thus, we first need to wait to get
  ; responses for all requested locks.
  (not (mutex (name ?res&:(member$ (mutex-to-resource ?res) ?req))
              (response PENDING)))
  =>
  (modify ?g (mode FINISHED) (outcome REJECTED) (message ?err))
  (do-for-all-facts
    ((?om mutex))
    (and (member$ ?om:response (create$ REJECTED ERROR))
         (member$ (mutex-to-resource ?om:name) ?req))
    (modify ?om (request NONE) (response NONE) (error-msg ""))
  )
)

(defrule resource-locks-unlock-start
  ?g <- (goal (mode RETRACTED) (acquired-resources $?acq))
  =>
  (foreach ?res ?acq
    (if (not (any-factp ((?m mutex))
                        (and (eq ?m:name (resource-to-mutex ?res))
                             (eq ?m:request UNLOCK))))
     then
      (printout warn "Unlocking resource " ?res crlf)
      (mutex-unlock-async (resource-to-mutex ?res))
    )
  )
)

(defrule resource-locks-unlock-done
  ?m <- (mutex (name ?res) (request UNLOCK) (response UNLOCKED))
  ?g <- (goal (acquired-resources $?acq
                &:(member$ (mutex-to-resource ?res) ?acq)))
  =>
  (modify ?g (acquired-resources
              (delete-member$ ?acq (mutex-to-resource ?res))))
  (modify ?m (request NONE) (response NONE))
)
