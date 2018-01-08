
;---------------------------------------------------------------------------
;  wm-config.clp - CLIPS executive
;
;  Created: Tue Jan 5 18:39  2017
;  Copyright  2012-2017  Mostafa Gomaa [gomaa@kbsg.rwth-aachen.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; (defglobal
;   ?*WM_CONFIG_PREFIX* = "?"
; )

; (defrule wm-config-compute-config-prefix
;   "Set the prefix of wm-config as a global"
;   (executive-init)
;   (confval (path "/clips-executive/spec") (type STRING) (value ?spec))
;   =>
;   (bind ?*WM_CONFIG_PREFIX* (str-cat "/clips-executive/specs/" ?spec "/wm-config/"))
; )

(defrule wm-config-load-from-confval
  "Convert worldmodel related confval into wm-fact representation."
  (executive-init)
  (confval (path "/clips-executive/spec") (type STRING) (value ?spec))
  ?conf <- (confval (path ?path&:(str-index (str-cat "/clips-executive/specs/" ?spec "/wm-config/") ?path)) 
  					(type ?type) (value ?value) (is-list ?is-list) (list-value $?list-value))
  =>
  (bind ?wm-config-prefix (str-cat "/clips-executive/specs/" ?spec "/wm-config/"))
  (bind ?wm-config-postfix (sub-string (str-length ?wm-config-prefix) (str-length ?path) ?path))
  (bind ?id (str-cat "/config" ?wm-config-postfix))
  (assert (wm-fact (id ?id) (type ?type) (value ?value) (is-list ?is-list) (values ?list-value) ) )
  (retract ?conf)
  )

