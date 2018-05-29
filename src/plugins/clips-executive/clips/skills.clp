
;---------------------------------------------------------------------------
;  skills.clp - CLIPS skill utilities
;
;  Created: Thu Dec 20 12:06:02 2012 (Train from Freiburg to Aachen)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
	?*SKILL-INIT-TIMEOUT-SEC* = 5
	?*SKILL-ACQUIRE-CONTROL-RETRY-INTERVAL-SEC* = 3
)

(deftemplate skill
	; Unique ID, meaningful only in the realm of this skill executor
	; and for components that interface with it.
	; It is guaranteed to be unique, also for the case when executin the
	; exact same skill a second time.
	(slot id (type SYMBOL))
  (slot name (type SYMBOL))
  (slot status (type SYMBOL) (allowed-values S_IDLE S_RUNNING S_FINAL S_FAILED))
  (slot error-msg (type STRING))
  (slot skill-string (type STRING))
	(slot msgid (type INTEGER))
  (multislot start-time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
)

(deftemplate skiller-control
	(slot acquired (type SYMBOL) (allowed-values FALSE TRUE))
	(slot acquiring (type SYMBOL) (allowed-values FALSE TRUE))
  (multislot last-try (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
)

; Expects that the file is batch-loaded
(blackboard-open-reading "SkillerInterface" "Skiller")
(assert (skiller-control))
(assert (ff-feature-loaded skills))

(deffunction merge-params (?params ?param-values)
  (if (not (= (length$ ?params) (length$ ?param-values))) then
    (printout logerror "Invalid skill call, number of parameters is not the "
                       "same as number of parameter values")
    (return FALSE)
  )
  (if (eq (length$ ?params) 0) then
    (return (create$))
  )
  (return (insert$ (merge-params (rest$ ?params) (rest$ ?param-values))
                   1
                   (first$ ?params) (first$ ?param-values))
  )
)

(deffunction skill-call (?name ?param-names ?param-values)
	; The following is a basic 1-to-1 mapping from action to skill
  ; (bind ?args (merge-params ?param-names ?param-values))
  ; (if (eq FALSE ?args) then
  ;   (printout logerror "Error constructing skill call, abort.")
  ;   (return FALSE)
  ; )
  ; (bind ?sks "")
  ; (foreach ?a ?args
	;    (if (is-odd-int ?a-index) then
	;      ; argument name
	;      (if (neq ?sks "") then (bind ?sks (str-cat ?sks ", ")))
	;      (bind ?sks (str-cat ?sks ?a "="))
	;      else
	;      (bind ?a-type (type ?a))
	;      (switch ?a-type
	; 	     (case STRING then
  ;            (if (eq (sub-string 1 1 ?a) "{") then
  ;              (bind ?sks (str-cat ?sks ?a))
  ;             else
  ;              (bind ?sks (str-cat ?sks "\"" ?a "\""))))
	; 	     (case SYMBOL then
	; 		   (if (or (eq ?a true) (eq ?a false)) then
	; 		     (bind ?sks (str-cat ?sks ?a))
	; 		    else
	; 		     (bind ?sks (str-cat ?sks "\"" ?a "\""))
	; 		   ))
	; 	     (default (bind ?sks (str-cat ?sks ?a)))
	;      )
	;    )
  ; )
  ; (bind ?sks (str-cat ?name "{" ?sks "}"))

	; And here we rely on a function provided from the outside providing
	; a more sophisticated mapping.
	(bind ?sks (map-action-skill ?name ?param-names ?param-values))
	(printout logwarn "sks='" ?sks "'" crlf)

	(bind ?id UNKNOWN)
	(if (eq ?sks "")
			then
		(bind ?id (sym-cat ?name (gensym*)))
		(assert (skill (id ?id) (name (sym-cat ?name)) (status S_FAILED) (start-time (now))
		        (error-msg (str-cat "Failed to convert action '" ?name "' to skill string"))))
	else
		(bind ?m (blackboard-create-msg "SkillerInterface::Skiller" "ExecSkillMessage"))
		(blackboard-set-msg-field ?m "skill_string" ?sks)

		(printout logwarn "Calling skill '" ?sks "'" crlf)
		(bind ?msgid (blackboard-send-msg ?m))
		(bind ?status (if (eq ?msgid 0) then S_FAILED else S_IDLE))
		(bind ?id (sym-cat ?name "-" ?msgid))
		(assert (skill (id ?id) (name (sym-cat ?name)) (skill-string ?sks)
									 (status ?status) (msgid ?msgid) (start-time (now))))
	)
	(return ?id)
)

(defrule skill-control-acquire
	(time $?now)
	?sc <- (skiller-control (acquired FALSE) (acquiring FALSE)
													(last-try $?lt&:(timeout ?now ?lt ?*SKILL-ACQUIRE-CONTROL-RETRY-INTERVAL-SEC*)))
	=>
	(printout t "Acquiring exclusive skiller control" crlf)
	(bind ?m (blackboard-create-msg "SkillerInterface::Skiller" "AcquireControlMessage"))

	(if (any-factp ((?c confval)) (and (eq ?c:path "/clips-executive/steal-skiller-control")
																		 (eq ?c:value TRUE)))
	then
		(blackboard-set-msg-field ?m "steal_control" TRUE)
	)
	(bind ?msgid (blackboard-send-msg ?m))
	(if (neq ?msgid 0)
	then
		(modify ?sc (acquiring TRUE) (last-try ?now))
	else
		(modify ?sc (last-try ?now))
	)
)

(defrule skill-control-acquired
	?sc <- (skiller-control (acquired FALSE) (acquiring TRUE))
	(blackboard-interface (type "SkillerInterface") (id "Skiller") (serial ?s))
  (SkillerInterface (id "Skiller") (exclusive_controller ?s))
	=>
	(printout t "Acquired exclusive skiller control" crlf)
	(modify ?sc (acquired TRUE) (acquiring FALSE))
)
		
(defrule skill-control-lost
	?sc <- (skiller-control (acquired TRUE))
	(blackboard-interface (type "SkillerInterface") (id "Skiller") (serial ?s))
  (SkillerInterface (id "Skiller") (exclusive_controller ~?s))
	=>
	(printout t "Lost exclusive skiller control" crlf)
	(modify ?sc (acquired FALSE))
)

(defrule skill-control-release
	(declare (salience 1000))
	(executive-finalize)
	?sc <- (skiller-control (acquired TRUE))
	?bi <- (blackboard-interface (type "SkillerInterface") (id "Skiller"))
	=>
	(printout t "Releasing control on finalize" crlf)
	(bind ?m (blackboard-create-msg "SkillerInterface::Skiller" "ReleaseControlMessage"))
	(blackboard-send-msg ?m)
	(blackboard-close "SkillerInterface" "Skiller")
	(modify ?sc (acquired FALSE))
	(retract ?bi)
)	

(defrule skill-status-update
  ?su <- (SkillerInterface (id "Skiller") (msgid ?msgid) (status ?new-status)
                           (error ?error-msg))
  ?sf <- (skill (name ?n) (msgid ?msgid) (status ?old-status&~?new-status))
  =>
  (printout t "Skill " ?n " is " ?new-status", was: " ?old-status crlf)
  (retract ?su)
  (modify ?sf (status ?new-status) (error-msg ?error-msg))
)

(defrule skill-status-update-nochange
  ?su <- (SkillerInterface (id "Skiller") (msgid ?msgid) (status ?new-status))
  (skill (name ?n) (msgid ?msgid) (status ?new-status))
  =>
  (retract ?su)
)

(defrule skill-start-timeout
	(time $?now)
  ?sf <- (skill (name ?n) (status S_IDLE)
								(start-time $?st&:(timeout ?now ?st ?*SKILL-INIT-TIMEOUT-SEC*)))
  =>
	(printout warn "Timeout starting skill " ?n " (" ?*SKILL-INIT-TIMEOUT-SEC* " sec): assuming failure" crlf)
	(modify ?sf (status S_FAILED) (error-msg "Start timeout"))
)

; Do not auto cleanup, the application invoking the skill should do that
;(defrule skill-idle
;  (declare (salience -4000))
;  ?sf <- (skill (status S_FINAL|S_FAILED))
;  =>
;  (retract ?sf)
;)

; Quick skill execution test
;(defrule skill-test
;	(start)
;	(skiller-control (acquired TRUE))
;	=>
;	(skill-call say (create$ text "Hello world" wait true))
;)
