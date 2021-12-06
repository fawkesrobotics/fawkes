;****************************************************************************
;  debug.pddl: Debug output for the test domain
;
;  Created: Thu Dec 2 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;****************************************************************************


(defrule debug-print-container-at
    (domain-fact (name container-at) (param-values ?c ?l))
    =>
    (printout t "DEBUG: Container " ?c " is now at " ?l crlf)
)

(defrule debug-print-robot-at
    (domain-fact (name robot-at) (param-values ?r ?l))
    =>
    (printout t "DEBUG: Robot " ?r " is now at " ?l crlf)
)

(defrule debug-print-machine-in-state
    (domain-fact (name machine-in-state) (param-values ?m ?s))
    =>
    (printout t "DEBUG: Machine " ?m " is now in state " ?s crlf)
)

(defrule debug-print-container-filled-with
    (domain-fact (name container-filled) (param-values ?c ?mat))
    =>
    (printout t "DEBUG: Container " ?c " is now filled with " ?mat crlf)
)

(defrule debug-assert-start-session-timer
    (time ?now ?mills)
    (not (scenario-timer ?))
    (goal (class PRODUCTION-RUN-ONE))
    =>
    (assert (scenario-timer ?now))
)

(defrule debug-assert-stop-condition-storage-full
    (domain-fact (name container-at) (param-values ?c STORAGE-INPUT))
    (not (domain-fact (name storage-is-full)))
    (time ?now ?mills)
    ?st <- (scenario-timer ?start)
    =>
    (bind ?dcounter 0)
    (do-for-all-facts ((?df domain-fact)) (eq ?df:name container-at)
        (bind ?dcounter (+ ?dcounter 1))
    )
    (bind ?ccounter 0)
    (do-for-all-facts ((?dc domain-constant)) (eq ?dc:type container)
        (bind ?ccounter (+ ?ccounter 1))
    )
    (if (eq ?dcounter ?ccounter) then
        (assert (domain-fact (name storage-is-full)))
        (do-for-all-facts ((?g goal))
            (retract ?g)
        )
        (printout t "DEBUG: Scenario fulfilled in " (- ?now ?start) "s!" crlf)
        (retract ?st)
    )
)