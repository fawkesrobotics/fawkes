;---------------------------------------------------------------------------
;  promises.clp - CLIPS executive - promise representation rules
;
;  Created: Tue Nov 23 2021 00:30:00
;  Copyright  2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate promise-config
  (slot enabled (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deftemplate domain-promise
  "A promise is like a domain-fact with the exception that it is not true yet."
  (slot name (type SYMBOL) (default ?NONE))
  (multislot param-values)
  (slot negated (type SYMBOL) (allowed-values TRUE FALSE))
  (slot promising-goal (type SYMBOL))
  (slot promising-agent (type SYMBOL) (default nil))
  (slot active (type SYMBOL) (default FALSE) (allowed-values TRUE FALSE))
  (slot valid-at (type INTEGER))
  (slot do-not-invalidate (type SYMBOL) (default FALSE));indicate if the promise should be retracted with its source goal
)

(deffunction goal-agent (?goal ?agent)
  (return (sym-cat ?goal @ ?agent))
)

(deftemplate promise-time
  (slot usecs (type INTEGER))
)

(deffunction sat-or-promised (?sat ?now ?from ?lt)
  (return (or
      (eq ?sat TRUE)
      (and
        (any-factp ((?pconfig promise-config)) ?pconfig:enabled)
        (<= ?now ?from)
        (< (- ?from ?now) ?lt)
        (neq ?from -1)
      )
    )
  )
)

(defrule promises-check-if-enabled
  (confval (path "/clips-executive/spec") (value ?spec))
  (confval
    (path ?path&:(eq ?path (str-cat "/clips-executive/specs/" ?spec
                                    "/parameters/use-promises")))
    (type BOOL)
    (value ?val))
  =>
  (printout t "Promises are " (if ?val then "enabled" else "disabled") crlf)
  (assert (promise-config (enabled ?val)))
)

(defrule promises-check-if-not-defined-in-config
  (confval (path "/clips-executive/spec") (value ?spec))
  (not (confval
    (path ?path&:(eq ?path (str-cat "/clips-executive/specs/" ?spec
                                    "/parameters/use-promises")))
    (type BOOL)))
  =>
  (printout t "Promises are disabled" crlf)
  (assert (promise-config (enabled FALSE)))
)

(defrule promises-load-files
  (promise-config (enabled TRUE))
 =>
  (path-load promises-formulas.clp)
  (path-load wm-promise-sync.clp)
)
