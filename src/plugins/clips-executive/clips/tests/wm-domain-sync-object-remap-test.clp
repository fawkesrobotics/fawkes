
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

(defglobal
	?*ID* =  "/some/other/text"
	?*SPEC* = "test"
	?*CONF-PATH* = (str-cat "/clips-executive/specs/" ?*SPEC* "/")
)

(printout t "***** ADD remapping *****" crlf)
;(assert (wm-sync-remap-object-type (domain-object-type text) (wm-fact-key some other object)))
;(wm-sync-remap-object-id-prefix ?*ID*)
(assert (executive-init)
				(confval (path "/clips-executive/spec") (type STRING) (value ?*SPEC*))
 				;(confval (path (str-cat ?*CONF-PATH* "wm-remap/objects/name-id/text"))
				;				 (type STRING) (value "/wm/words"))
				(confval (path (str-cat ?*CONF-PATH* "wm-remap/objects/id-prefix")) (type STRING) (is-list TRUE)
											 (list-value "/wm/text"))
)
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
(do-for-fact ((?wf wm-fact)) (eq ?wf:id ?*ID*)
	(modify ?wf (values hello goodbye sometext))
)
(run)
(facts)
(printout t "***** RETRACT worldmodel *****" crlf)
(do-for-fact ((?wf wm-fact)) (eq ?wf:id ?*ID*)
	(modify ?wf (values hello sometext))
)
(run)
(facts)

(do-for-fact ((?wf wm-fact)) (eq ?wf:id ?*ID*)
	(modify ?wf (values))
	;(retract ?wf)
)
(run)
(facts)

(exit)
