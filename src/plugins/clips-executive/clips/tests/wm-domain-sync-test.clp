
; Start with echo "(batch wm-domain-sync-test.clp)" | clips | less

(load* "../../../clips/clips/utils.clp")
(load* "../../../clips/clips/ff-config.clp")
(load* "../plan.clp")
(load* "../domain.clp")
(load* "../worldmodel.clp")
(load* "../wm-domain-sync.clp")

(watch facts)
(watch rules)

(deffacts testfacts
	(domain-loaded)
	(domain-object-type (name text))
	(domain-object (name hello) (type text))
	(domain-object (name goodbye) (type text))
	(domain-object (name sometext) (type text))
	(domain-predicate (name said) (param-names c) (param-types text))
)

(printout t "***** RESET *****" crlf)
(reset)
(run)
(facts)

(printout t "***** ASSERT domain *****" crlf)
(assert (domain-fact (name said) (param-values hello)))
(run)
(printout t "***** RETRACT domain *****" crlf)
(do-for-fact ((?df domain-fact)) (retract ?df))
(run)
(facts)

(printout t "***** ASSERT domain *****" crlf)
(assert (domain-fact (name said) (param-values hello)))
(run)
(printout t "***** SET wm-sync NO retract *****" crlf)
(wm-sync-set-wm-fact-retract "/domain/fact/said?c=hello" FALSE)
(facts)
(printout t "***** RETRACT domain *****" crlf)
(do-for-fact ((?df domain-fact)) (retract ?df))
(run)
(facts)

(printout t "***** ASSERT domain 2 *****" crlf)
(assert (domain-fact (name said) (param-values hello)))
(run)
(printout t "***** SET wm-sync DO retract *****" crlf)
(wm-sync-set-wm-fact-retract "/domain/fact/said?c=hello" TRUE)
(printout t "***** RETRACT domain 2 *****" crlf)
(do-for-fact ((?df domain-fact)) (retract ?df))
(run)
(facts)

(printout t "***** ASSERT wm *****" crlf)
(assert (wm-fact (id "/domain/fact/said?c=goodbye") (type BOOL) (value TRUE)))
(run)

(printout t "***** SET FALSE wm *****" crlf)
(do-for-fact ((?wf wm-fact)) (modify ?wf (value FALSE)))
(run)
(facts)

(printout t "***** SET TRUE wm *****" crlf)
(do-for-fact ((?wf wm-fact)) (modify ?wf (value TRUE)))
(run)
(facts)

(printout t "***** RETRACT wm *****" crlf)
(do-for-fact ((?wf wm-fact)) (retract ?wf))
(run)
(facts)

(exit)
