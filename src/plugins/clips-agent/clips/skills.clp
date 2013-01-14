
;---------------------------------------------------------------------------
;  skills.clp - CLIPS skill utilities
;
;  Created: Thu Dec 20 12:06:02 2012 (Train from Freiburg to Aachen)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate skill
  (slot name (type STRING))
  (slot status (type SYMBOL) (allowed-values IDLE RUNNING FINAL FAILED))
  (slot skill-string (type STRING))
)

(deftemplate skill-update
  (slot name (type STRING))
  (slot status (type SYMBOL) (allowed-values IDLE RUNNING FINAL FAILED))
)

(deffunction skill-call (?name $?args)
  (if (is-odd-int (length$ ?args)) then
    (printout logerror "Invalid skill call, number of arguments must be even" crlf)
    (return FALSE)
  )
  (bind ?sks "")
  (foreach ?a ?args
	   (if (is-odd-int ?a-index) then
	     ; argument name
	     (if (neq ?sks "") then (bind ?sks (str-cat ?sks ", ")))
	     (bind ?sks (str-cat ?sks ?a "="))
	     else
	     (bind ?a-type (type ?a))
	     (switch ?a-type
		     (case STRING then (bind ?sks (str-cat ?sks "\"" ?a "\"")))
		     (case SYMBOL then
			   (if (or (eq ?a true) (eq ?a false)) then
			     (bind ?sks (str-cat ?sks ?a))
			    else
			     (bind ?sks (str-cat ?sks "\"" ?a "\""))
			   ))
		     (default (bind ?sks (str-cat ?sks ?a)))
	     )
	   )
  )
  (bind ?sks (str-cat ?name "{" ?sks "}"))
  (printout logwarn "Calling skill " ?sks crlf)
  (assert (skill (name (str-cat ?name)) (status IDLE) (skill-string ?sks)))
  (skill-call-ext ?name ?sks)
)


(defrule skill-update
  ?su <- (skill-update (name ?n) (status ?new-status))
  ?sf <- (skill (name ?n) (status ?old-status&~?new-status))
  =>
  (printout t "Skill " ?n " is " ?new-status", was: " ?old-status crlf)
  (retract ?su)
  (modify ?sf (status ?new-status))
)

(defrule skill-update-nochange
  ?su <- (skill-update (name ?n) (status ?new-status))
  (skill (name ?n) (status ?new-status))
  =>
  (retract ?su)
)
