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

(defrule resource-locks-request-lock
  (goal (mode COMMITTED)
        (acquired-resources $?acq)
        (required-resources $?req&:(set-diff ?req ?acq)))
  (not (mutex (name ?res) (request ~NONE)))
  =>
  (foreach ?res (set-diff ?req ?acq)
    (if (not (any-factp ((?m mutex)) (neq ?m:request NONE))) then
      (printout warn "Trying to lock " ?res crlf)
      (mutex-try-lock-async ?res)
    )
  )
)

(defrule resource-locks-lock-acquired
  ?m <- (mutex (name ?res) (request LOCK) (response ACQUIRED))
  ?g <- (goal (mode COMMITTED)
              (required-resources $?req)
              (acquired-resources $?acq&:(member$ ?res (set-diff ?req ?acq))))
  =>
  (modify ?g (acquired-resources (append$ ?acq ?res)))
  (modify ?m (request NONE) (response NONE))
)

(defrule resource-locks-lock-rejected
  ?m <- (mutex (name ?res)
               (request LOCK)
               (response REJECTED|ERROR)
               (error-msg ?err))
  ?g <- (goal (mode COMMITTED)
              (required-resources $?req)
              (acquired-resources $?acq&:(member$ ?res (set-diff ?req ?acq))))
  =>
  (modify ?g (mode FINISHED) (outcome REJECTED) (message ?err))
  (modify ?m (request NONE) (response NONE) (error-msg ""))
)

(defrule resource-locks-unlock-start
  ?g <- (goal (mode RETRACTED) (acquired-resources $?acq))
  =>
  (foreach ?res ?acq
    (if (not (any-factp ((?m mutex)) (neq ?m:request NONE))) then
      (printout warn "Trying to unlock " ?res crlf)
      (mutex-unlock-async ?res)
    )
  )
)

(defrule resource-locks-unlock-done
  ?m <- (mutex (name ?res) (request UNLOCK) (response UNLOCKED))
  ?g <- (goal (mode RETRACTED)
              (acquired-resources $?acq&:(member$ ?res ?acq)))
  =>
  (modify ?g (acquired-resources (delete-member$ ?acq ?res)))
  (modify ?m (request NONE) (response NONE))
)
