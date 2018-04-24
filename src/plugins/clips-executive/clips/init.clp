
;---------------------------------------------------------------------------
;  init.clp - CLIPS executive
;
;  Created: Tue Sep 19 16:49:42 2017
;  Copyright  2012-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*CONFIG_PREFIX* = "/clips-executive"
	?*INIT-STAGES* = (create$ STAGE-1 STAGE-2 STAGE-3)
	?*CX-STAGE2-FILES* = (create$ "plan.clp" "domain.clp" "worldmodel.clp" "wm-domain-sync.clp"
	                              "wm-config.clp" "BATCH|skills.clp")
	?*CX-USER-INIT-OFFSET* = 10
)

(deftemplate executive-init-request
	(slot stage (type SYMBOL))
	(slot name (type SYMBOL))
	(slot order (type INTEGER))
	(slot feature)
	(multislot files (type STRING))
	(slot state (type SYMBOL) (allowed-values PENDING FEATURE-REQUESTED FEATURE-DONE COMPLETED ERROR))
	(multislot error-msgs (type STRING))
)

(defrule executive-load-config
  (declare (salience ?*SALIENCE-INIT*))
  (executive-init)
  =>
  (config-load ?*CONFIG_PREFIX*)
)

(deffunction cx-debug-unwatch-facts ($?templates)
	(bind ?deftemplates (get-deftemplate-list))
	(printout debug "Unwatching fact templates " ?templates crlf) 
	(foreach ?v ?templates
		(bind ?v-sym (sym-cat ?v))
		(if (member$ ?v-sym ?deftemplates)
		 then (unwatch facts ?v-sym)
		 else (printout warn "Cannot unwatch " ?v " (deftemplate not defined)" crlf)
		 )
	)
)

(deffunction cx-debug-unwatch-rules ($?rules)
	(bind ?defrules (get-defrule-list))
	(printout debug "Unwatching rules " ?rules crlf) 
	(foreach ?v ?rules
		(bind ?v-sym (sym-cat ?v))
		(if (member$ ?v-sym ?defrules)
		 then (unwatch rules ?v-sym)
		 else (printout warn "Cannot unwatch " ?v " (defrule not defined)" crlf)
		)
	)
)

(defrule executive-enable-debug
  (declare (salience ?*SALIENCE-INIT*))
  (executive-init)
  (confval (path "/clips-executive/debug/enable") (type BOOL) (value TRUE))
  (confval (path "/clips-executive/spec") (type STRING) (value ?spec))
  =>
  (printout t "CLIPS debugging enabled, watching facts and rules" crlf)
  (watch facts)
  (watch rules)
  ;(dribble-on "trace.txt")
	(do-for-fact ((?c confval)) (and (eq ?c:path "/clips-executive/debug/level") (eq ?c:type UINT))
		(printout debug "Setting debug level to " ?c:value " (was " ?*DEBUG* ")" crlf)
		(debug-set-level ?c:value)
	)

	(do-for-fact ((?c confval)) (and (eq ?c:path "/clips-executive/debug/unwatch-facts")
																	 (eq ?c:type STRING) ?c:is-list)
	 (cx-debug-unwatch-facts ?c:list-value)
  )
	(do-for-fact ((?c confval)) (and (eq ?c:path "/clips-executive/debug/unwatch-rules")
																	 (eq ?c:type STRING) ?c:is-list)
	 (cx-debug-unwatch-rules ?c:list-value)
  )
)

(deffunction cx-init-indexes (?spec ?stage)
	(bind ?rv (create$))
	(do-for-all-facts ((?c confval)) (str-prefix (str-cat "/clips-executive/specs/" ?spec "/init/" ?stage "/")
																							 ?c:path)
		(bind ?path-elements (str-split ?c:path "/"))
		(bind ?idx (nth$ 6 ?path-elements))
		(if (not (member$ ?idx ?rv)) then	(bind ?rv (append$ ?rv ?idx)))
	)
	(return ?rv)
)

(deffunction cx-assert-init-requests (?spec ?stage ?feature-default)
	(bind ?cfg-stage (str-cat (lowcase ?stage)))
	(bind ?cfgpfx (str-cat "/clips-executive/specs/" ?spec "/init/" ?cfg-stage "/"))
	(foreach ?i (cx-init-indexes ?spec ?cfg-stage)
		(bind ?feature ?feature-default)
		(bind ?name "MISSING")
		(bind ?files (create$))
		(bind ?error-msgs (create$))
		(bind ?state PENDING)
		(if (not (any-factp ((?c confval)) (eq (str-cat ?cfgpfx ?i "/name") ?c:path)))
		 then
			(bind ?state ERROR)
			(bind ?error-msgs (append$ ?error-msgs (str-cat ?stage " entry " ?i " is missing name entry")))
		 else
			(do-for-fact ((?c confval)) (eq (str-cat ?cfgpfx ?i "/name") ?c:path)
				(bind ?name ?c:value)
			)
			(do-for-fact ((?c confval)) (eq (str-cat ?cfgpfx ?i "/feature-request") ?c:path)
				(bind ?feature ?c:value)
			)
			(do-for-fact ((?c confval)) (eq (str-cat ?cfgpfx ?i "/file") ?c:path)
				(bind ?files (append$ ?files ?c:value))
			)
			(do-for-fact ((?c confval)) (eq (str-cat ?cfgpfx ?i "/files") ?c:path)
				(if (not ?c:is-list)
				 then
					(printout warn "Config entry " (str-cat ?cfgpfx ?i "/files") " is not a list value, ignoring")
				 else
					(bind ?files (append$ ?files ?c:list-value))
				)
			)
		)
		(assert (executive-init-request (state ?state) (error-msgs ?error-msgs)
																		(stage ?stage) (order (+ ?i-index ?*CX-USER-INIT-OFFSET*))
																		(name (sym-cat ?name)) (feature ?feature)
																		(files ?files)))
	)
)

(defrule executive-init-start
  (declare (salience ?*SALIENCE-INIT-LATE*))
	(executive-init)
  (confval (path "/clips-executive/spec") (type STRING) (value ?spec))
	=>
	(cx-assert-init-requests ?spec STAGE-1 TRUE)
	(assert (executive-init-request (state PENDING)	(stage STAGE-2) (order 0)
																	(name cx-files) (feature FALSE) (files ?*CX-STAGE2-FILES*)))
	(cx-assert-init-requests ?spec STAGE-2 FALSE)
	(cx-assert-init-requests ?spec STAGE-3 FALSE)
	(assert (executive-init-stage STAGE-1))
)

(defrule executive-init-failed
  (declare (salience ?*SALIENCE-INIT*))
	(executive-init)
	(executive-init-request (state ERROR) (stage ?stage) (order ?i) (error-msgs $?error-msgs))
	?sf <- (executive-init-stage ?stage)
	=>
	(printout error crlf)
	(printout error "***********************************************************" crlf)
	(printout error crlf)
	(printout error ?stage " request " ?i " failed: " ?error-msgs crlf)
	(printout error crlf)
	(printout error "***********************************************************" crlf)
	(printout error crlf)
	(retract ?sf)
	(assert (executive-init-stage FAILED))
)

(defrule executive-init-stage-request-feature
	(executive-init)
	(executive-init-stage ?stage)
	?ir <- (executive-init-request (state PENDING) (stage ?stage) (order ?order)
																 (name ?name) (feature TRUE))
	(not (executive-init-request (state ~COMPLETED) (stage ?stage) (order ?order2&:(< ?order2 ?order))))
	(ff-feature ?name)
	=>
	(printout t "Init " ?stage ": requesting feature " ?name crlf)
	(ff-feature-request (str-cat ?name))
	(modify ?ir (state FEATURE-REQUESTED))
)

(defrule executive-init-stage-request-no-feature
	(executive-init)
	(executive-init-stage ?stage)
	?ir <- (executive-init-request (state PENDING) (stage ?stage) (order ?order)
																 (name ?name) (feature FALSE))
	(not (executive-init-request (state ~COMPLETED) (stage ?stage) (order ?order2&:(< ?order2 ?order))))
	=>
	(modify ?ir (state FEATURE-DONE))
)

(defrule executive-init-stage-request-feature-unavailable
	(executive-init)
	(executive-init-stage ?stage)
	?ir <- (executive-init-request (state PENDING) (stage ?stage) (order ?order)
																 (name ?name) (feature TRUE))
	(not (executive-init-request (state ~COMPLETED) (stage ?stage) (order ?order2&:(< ?order2 ?order))))
	;(not (ff-feature ?name))
	=>
	(printout error "Init " ?stage ": feature " ?name " is not available" crlf)
	(modify ?ir (state ERROR) (error-msgs (str-cat "Feature " ?name " is not available")))
)

(defrule executive-init-stage-request-feature-fulfilled
	(executive-init)
	(executive-init-stage ?stage)
	?ir <- (executive-init-request (state FEATURE-REQUESTED) (stage ?stage) (name ?name))
	(ff-feature-loaded ?name)
	=>
	(printout debug "Init " ?stage ": feature request for " ?name " has been fulfilled" crlf)
	(modify ?ir (state FEATURE-DONE))
)

(defrule executive-init-stage-request-files
	(executive-init)
	(executive-init-stage ?stage)
	?ir <- (executive-init-request (state FEATURE-DONE) (stage ?stage) (name ?name) (files $?files))
	=>
	(if (> (length$ ?files) 0)
	 then
		(printout t "Init " ?stage ": loading files for " ?name " " ?files crlf)
		(foreach ?f ?files
			(bind ?pipepos (str-index "|" ?f))
			(bind ?file-op "LOAD")
			(bind ?file-name ?f)
			(if ?pipepos then
				(bind ?file-op (sub-string 1 (- ?pipepos 1) ?f))
				(bind ?file-name (sub-string (+ ?pipepos 1)  (str-length ?f) ?f))
			)
			(switch ?file-op
				(case "BATCH" then  (path-batch* ?file-name))
				(case "BATCH*" then (path-batch* ?file-name))
				(default (path-load ?file-name))
			)
		)
	)
	(modify ?ir (state COMPLETED))
)

(defrule executive-init-stage-finished
	(executive-init)
	?sf <- (executive-init-stage ?stage)
	(not (executive-init-request (state ~COMPLETED) (stage ?stage)))
	=>
	(retract ?sf)
	(bind ?stage-idx (member$ ?stage ?*INIT-STAGES*))
	(if (< ?stage-idx (length$ ?*INIT-STAGES*))
	 then
		(bind ?next-stage (nth$ (+ ?stage-idx 1) ?*INIT-STAGES*))
		(printout t "Init " ?stage ": finished, advancing to " ?next-stage crlf)
		(assert (executive-init-stage ?next-stage))
	 else
	 (printout t "Initialization completed" crlf)
	 (assert (executive-initialized))
	)
)
