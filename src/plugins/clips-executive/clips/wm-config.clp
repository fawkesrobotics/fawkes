
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
	=>
  (do-for-all-facts ((?cf-spec confval) (?cf confval))
    (and (eq ?cf-spec:path "/clips-executive/spec")
         (str-prefix (str-cat "/clips-executive/specs/" ?cf-spec:value "/parameters/") ?cf:path))
    (bind ?prefix (str-cat "/clips-executive/specs/" ?cf-spec:value "/parameters/"))
    (bind ?name (sub-string (str-length ?prefix) (str-length ?cf:path) ?cf:path))
    (bind ?id (str-cat "/config" ?name))
    (assert (wm-fact (id ?id) (type ?cf:type) (value ?cf:value) (is-list ?cf:is-list)
                     (values ?cf:list-value)))
  )
  (bind ?prefix "/fawkes/agent/")
  (do-for-all-facts ((?cf confval))
    (str-prefix "/fawkes/agent/" ?cf:path)
    (bind ?name (sub-string (str-length ?prefix) (str-length ?cf:path) ?cf:path))
    (bind ?id (str-cat "/config/agent" ?name))
    (assert (wm-fact (id ?id) (type ?cf:type) (value ?cf:value)
            (is-list ?cf:is-list)
            (values ?cf:list-value)))
  )
)
