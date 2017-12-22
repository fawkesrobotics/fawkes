
;---------------------------------------------------------------------------
;  wm-domain-sync.clp - CLIPS executive - sync world and domain model
;
;  Created: Fri Dec 22 15:42:33 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate wm-sync-map
	(slot wm-fact-id (type STRING))
	(slot domain-fact-name (type SYMBOL))
	(slot domain-fact-idx (type INTEGER))
)

(defrule wm-sync-add-map-entry
	(domain-predicate (name ?name) (param-names $?param-names))
	?df <- (domain-fact (name ?name) (param-values $?param-values))
	(not (wm-sync-map (domain-fact-name ?name)
										(wm-fact-id ?id&:(eq ?id (wm-key-to-id domain fact ?name args?
																													 (domain-fact-key ?param-names ?param-values))))))
	=>
	(assert (wm-sync-map (wm-fact-id (wm-key-to-id domain fact ?name args?
																								 (domain-fact-key ?param-names ?param-values)))
											 (domain-fact-name ?name)
											 (domain-fact-idx (fact-index ?df))))
)

(defrule wm-sync-domain-fact-added
	(wm-sync-map (wm-fact-id ?id) (domain-fact-name ?name) (domain-fact-idx ?idx))
	(domain-predicate (name ?name) (param-names ?param-names))
	?df <- (domain-fact (name ?name) (param-values ?param-values))
	(test (eq ?idx (fact-index ?df)))
	(not (wm-fact (id ?id)))
	=>
	(assert (wm-fact (id ?id) (value TRUE)))
)

; Here, we assume that the param-names and the arguments in the key
; have the same ordering.
(deffunction wm-sync-key-arg-values ($?key)
	(bind ?rv (create$))
	(bind ?l (member$ args? ?key))
	(if ?l then
		(bind ?L (length$ ?key))
		(bind ?l (+ ?l 1))
		(while (<= (+ ?l 1) ?L) do
			;(bind ?arg (nth$ ?l ?key))
			(bind ?value (nth$ (+ ?l 1) ?key))
			(if (and (eq ?value [) (<= (+ ?l 2) ?L)) then
				(printout error "Domain predicates may not contain argument value list" crlf)
				(return FALSE)
			)
			(bind ?rv (append$ ?rv ?value))
			(bind ?l (+ ?l 2))
		)
	)
	(return ?rv)
)

(defrule wm-sync-domain-fact-removed
	(wm-sync-map (wm-fact-id ?id) (domain-fact-name ?name) (domain-fact-idx ?idx))
	?wf <- (wm-fact (id ?id) (key $?key) (value TRUE))
	(not (domain-fact (name ?name) (param-values $?param-values&:(eq ?param-values (wm-sync-key-arg-values ?key)))))
	=>
	(printout t "Key: " ?key "   Proc: " (wm-sync-key-arg-values ?key) crlf)
	(modify ?wf (value FALSE))
)
