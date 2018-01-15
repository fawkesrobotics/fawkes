
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

(defglobal
	?*ID* = "/wm/test/different"
	?*SPEC* = "test"
	?*CONF-PATH* = (str-cat "/clips-executive/specs/" ?*SPEC* "/")
)

(printout t "***** ADD remapping *****" crlf)
;(assert (wm-sync-remap-fact (domain-fact-name said) (wm-fact-key-path (wm-id-to-key ?*ID*))))
;(wm-sync-remap-fact-id-prefix ?*ID*)
(assert (executive-init)
				(confval (path "/clips-executive/spec") (type STRING) (value ?*SPEC*))
				(confval (path (str-cat ?*CONF-PATH* "wm-remap/facts/name-id/said"))
								 (type STRING) (value "/wm/spoken"))
				;(confval (path (str-cat ?*CONF-PATH* "wm-remap/facts/id-prefix")) (type STRING) (is-list TRUE)
				;							 (list-value "/wm/said"))
)
(run)

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
(wm-sync-set-wm-fact-retract (str-cat ?*ID* "?c=hello") FALSE)
(printout t "***** RETRACT domain *****" crlf)
(do-for-fact ((?df domain-fact)) (retract ?df))
(run)
(facts)

(printout t "***** ASSERT domain 2 *****" crlf)
(assert (domain-fact (name said) (param-values hello)))
(run)
(printout t "***** SET wm-sync DO retract *****" crlf)
(wm-sync-set-wm-fact-retract (str-cat ?*ID* "?c=hello") TRUE)
(printout t "***** RETRACT domain 2 *****" crlf)
(do-for-fact ((?df domain-fact)) (retract ?df))
(run)
(facts)

(printout t "***** ASSERT wm *****" crlf)
(assert (wm-fact (id (str-cat ?*ID* "?c=goodbye")) (type BOOL) (value TRUE)))
(run)
(facts)

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
