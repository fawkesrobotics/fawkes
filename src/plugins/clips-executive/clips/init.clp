
;---------------------------------------------------------------------------
;  init.clp - CLIPS executive
;
;  Created: Tue Sep 19 16:49:42 2017
;  Copyright  2012-2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*CONFIG_PREFIX* = "/clips-executive"
)

(defrule executive-load-config
  (executive-init)
  =>
  (config-load ?*CONFIG_PREFIX*)
)

(defrule executive-load-executive
  (executive-init)
  (confval (path "/clips-executive/spec") (type STRING) (value ?v))
  =>
  (printout t "Loading executive spec '" ?v "'" crlf)
  (bind ?executive-file (path-resolve (str-cat ?v ".clp")))
  (if ?executive-file
    then (batch* ?executive-file)
    else (printout logerror "Cannot find executive spec " ?v crlf))
)

(defrule executive-enable-debug
  (executive-init)
  (confval (path "/clips-executive/clips-debug") (type BOOL) (value TRUE))
  =>
  (printout t "CLIPS debugging enabled, watching facts and rules" crlf)
  (watch facts)
  (watch rules)
  ;(dribble-on "trace.txt")
)

(defrule executive-debug-level
  (executive-init)
  (confval (path "/clips-executive/debug-level") (type UINT) (value ?v))
  =>
  (printout t "Setting debug level to " ?v " (was " ?*DEBUG* ")" crlf)
  (debug-set-level ?v)
)

(defrule executive-silence-debug-facts
  (declare (salience -1000))
  (executive-init)
  (confval (path "/clips-executive/clips-debug") (type BOOL) (value TRUE))
  (confval (path "/clips-executive/unwatch-facts") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following facts: " ?lv crlf)
  (bind ?deftemplates (get-deftemplate-list))
  (foreach ?v ?lv
    (bind ?v-sym (sym-cat ?v))
    (if (member$ ?v-sym ?deftemplates)
     then (unwatch facts ?v-sym)
     else (printout warn "Cannot unwatch " ?v " (deftemplate not defined)" crlf)
    )
  )
)

(defrule executive-silence-debug-rules
  (declare (salience -1000))
  (executive-init)
  (confval (path "/clips-executive/clips-debug") (type BOOL) (value TRUE))
  (confval (path "/clips-executive/unwatch-rules") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following rules: " ?lv crlf)
  (bind ?defrules (get-defrule-list))
  (foreach ?v ?lv
    (bind ?v-sym (sym-cat ?v))
    (if (member$ ?v-sym ?defrules)
     then (unwatch rules ?v-sym)
     else (printout warn "Cannot unwatch " ?v " (defrule not defined)" crlf)
    )
  )
)

(defrule executive-init-skills
	(executive-init)
	=>
  (printout t "Blackboard feature and skill exec init" crlf)
	(ff-feature-request "blackboard")
)

(defrule executive-blackboard-feature-loaded
	(executive-init)
	(ff-feature blackboard)
	=>
  (printout t "Blackboard feature and skill exec init" crlf)
	(path-load "blackboard-init.clp")
	(path-load "skills-init.clp")
)
