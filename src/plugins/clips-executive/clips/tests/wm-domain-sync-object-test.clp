
; Start with echo "(batch wm-domain-sync-object-test.clp)" | clips | less

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
	;(domain-object (name hello) (type text))
	;(domain-object (name goodbye) (type text))
	;(domain-object (name sometext) (type text))
	;(domain-predicate (name said) (param-names c) (param-types text))
)

(printout t "***** RESET *****" crlf)
(reset)
(run)
(facts)

(printout t "***** ASSERT domain *****" crlf)
(assert (domain-object (name hello) (type text)))
(run)
(facts)
(printout t "***** RETRACT domain *****" crlf)
(do-for-fact ((?df domain-object)) (retract ?df))
(run)
(facts)

(printout t "***** ASSERT worldmodel *****" crlf)
(do-for-fact ((?wf wm-fact)) (eq ?wf:id "/domain/objects-by-type/text")
	(modify ?wf (values hello goodbye sometext))
)
(run)
(facts)
(printout t "***** RETRACT worldmodel *****" crlf)
(do-for-fact ((?wf wm-fact)) (eq ?wf:id "/domain/objects-by-type/text")
	(modify ?wf (values hello sometext))
)
(run)
(facts)

(do-for-fact ((?wf wm-fact)) (eq ?wf:id "/domain/objects-by-type/text")
	(modify ?wf (values))
	;(retract ?wf)
)
(run)
(facts)

(exit)
