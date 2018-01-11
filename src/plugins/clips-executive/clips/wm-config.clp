
;---------------------------------------------------------------------------
;  wm-config.clp - CLIPS executive
;
;  Created: Tue Jan 5 18:39  2017
;  Copyright  2012-2017  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule wm-config-load-from-confval
  "Convert parameters from configuration to world model facts"
	(executive-init)
  (confval (path "/clips-executive/spec") (type STRING) (value ?spec))
  ?cf <- (confval (path ?path&:(str-prefix (str-cat "/clips-executive/specs/" ?spec "/parameters/") ?path))
									(type ?type) (value ?value) (is-list ?is-list) (list-value $?list-value))
	=>
  (bind ?prefix (str-cat "/clips-executive/specs/" ?spec "/parameters/"))
  (bind ?name (sub-string (str-length ?prefix) (str-length ?path) ?path))
  (bind ?id (str-cat "/config" ?name))
  (assert (wm-fact (id ?id) (type ?type) (value ?value) (is-list ?is-list) (values ?list-value)))
  (retract ?cf)
)
