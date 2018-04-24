
;---------------------------------------------------------------------------
;  time.clp - time utils
;
;  Created: Sat Jun 16 15:45:57 2012 (Mexico City)
;  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
;             2011       SRI International
;             2011       RWTH Aachen University (KBSG)
;             2011       Carnegie Mellon University
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

;(defmodule TIME-UTILS)

(defglobal
  ?*PRIORITY-TIME-RETRACT*    = -10000
)

; This assumes Fawkes-style time, i.e. sec and usec

(deffunction time-diff (?t1 ?t2)
  (bind ?sec  (- (nth$ 1 ?t1) (nth$ 1 ?t2)))
  (bind ?usec (- (nth$ 2 ?t1) (nth$ 2 ?t2)))
  (if (< ?usec 0)
      then (bind ?sec (- ?sec 1)) (bind ?usec (+ 1000000 ?usec)))
  ;(printout t "time-diff called: " ?t1 " - " ?t2 " = " (create$ ?sec ?usec) crlf)
  (return (create$ ?sec ?usec))
)

(deffunction time-diff-sec (?t1 ?t2)
  (bind ?td (time-diff ?t1 ?t2))
  (return (+ (float (nth$ 1 ?td)) (/ (float (nth$ 2 ?td)) 1000000.)))
)

(deffunction timeout (?now ?time ?timeout)
  (return (> (time-diff-sec ?now ?time) ?timeout))
)

(deffunction time> (?t1 ?t2)
	(bind ?rv FALSE)
	(if (> (nth$ 1 ?t1) (nth$ 1 ?t2)) then (bind ?rv TRUE))
	(if (and (= (nth$ 1 ?t1) (nth$ 1 ?t2)) (> (nth$ 2 ?t1) (nth$ 2 ?t2))) then (bind ?rv TRUE))
	(return ?rv)
)


; Timer deftemplate, to be used with the timeout function.
; An example for a periodically triggered rule (assuming that you do have
; periodic assertion of a (time (now)) fact at suitable intervals) could be:
; @code
; (defrule timer-trigger
;   (time $?now)
;   ?tf <- (timer (name my-timer) (time $?t&:(timeout ?now ?t ?*TIMER-PERIOD*)) (seq ?seq))
;   =>
;   (modify ?tf (time ?now) (seq (+ ?seq 1)))
; )
; @endcode
; This triggers the rule every ?*TIMER-PERIOD* seconds and updates the
; last trigger time and the sequence number (which you can also skip if you
; never need it yourself.
(deftemplate timer
  (slot name)
  (multislot time (type INTEGER) (cardinality 2 2) (default-dynamic (now)))
  (slot seq (type INTEGER) (default 1))
)


; --- RULES - general housekeeping
(defrule time-retract
  (declare (salience ?*PRIORITY-TIME-RETRACT*))
  ?f <- (time $?)
  =>
  (retract ?f)
)
