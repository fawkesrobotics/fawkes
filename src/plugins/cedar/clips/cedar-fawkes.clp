
;---------------------------------------------------------------------------
;  fawkes.clp - Fawkes system integration for CLIPS
;
;  Created: Thu Dec 12 19:13:45 2013
;  Copyright  2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal ;FAWKES-HEALTH
  ?*FAWKES-COLLECT-PERIOD* = 1.0
)

(deftemplate fawkes-plugin
  (slot name (type STRING))
  (slot state (type SYMBOL) (allowed-values LOADED AVAILABLE)))
)

(deffunction fawkes-cleanup ()
  (delayed-do-for-all-facts ((?f fawkes-plugin)) TRUE (retract ?f))
  (delayed-do-for-all-facts ((?f blackboard-interface-info)) TRUE (retract ?f))
)

(deffacts fawkes-facts
  (system-condition (id fawkes))
  (timer (name fawkes-collect))
)


; --- RULES - data collection and cleanup
(defrule fawkes-collect
  (time $?now)
  ?tf <- (timer (name fawkes-collect) (time $?t&:(timeout ?now ?t ?*FAWKES-COLLECT-PERIOD*)))
  =>
  (modify ?tf (time ?now))
  (printout t "Requesting Fawkes blackboard info" crlf)
  (blackboard-get-info)
  (fawkes-get-plugin-info)
  (assert (fawkes-collect-process ?now))
)

(defrule fawkes-collect-cleanup
  (declare (salience -10000))
  ?rcpf <- (fawkes-collect-process $?)
  =>
  (retract ?rcpf)
  (fawkes-cleanup)
)
