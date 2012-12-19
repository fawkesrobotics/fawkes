
;---------------------------------------------------------------------------
;  config.clp - CLIPS agent configuration tools
;
;  Created: Wed Dec 19 20:45:53 2012 (Train from Munich to Freiburg)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate confval
  (slot path (type STRING))
  (slot type (type SYMBOL) (allowed-values FLOAT UINT INT BOOL STRING))
  (slot value)
)

(defrule print-confval
  (confval (path ?p) (type ?t) (value ?v))
  =>
  (if (debug 2) then
    (printout t "confval path: " ?p "  type: " ?t "  value: " ?v crlf))
)
