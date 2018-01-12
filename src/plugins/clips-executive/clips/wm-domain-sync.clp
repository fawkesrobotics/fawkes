
;---------------------------------------------------------------------------
;  wm-domain-sync.clp - CLIPS executive - sync world and domain model
;
;  Created: Fri Dec 22 15:42:33 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate wm-sync-map
	(slot wm-fact-id (type STRING))
	(multislot wm-fact-key (type SYMBOL))
	(slot wm-fact-idx (type INTEGER))
	(slot wm-fact-retract (type SYMBOL) (allowed-values TRUE FALSE) (default TRUE))
	(slot domain-fact-name (type SYMBOL))
	(slot domain-fact-idx (type INTEGER))
	)

(deffunction wm-sync-set-wm-fact-retract (?id ?enabled)
	"Allow to set behavior of wm-fact removal on domain-fact removal.
   If wm-fact retraction is enabled (wm-fact-retract slot in wm-sync-map TRUE),
   then a wm-fact is removed when the corresponding domain-fact is removed.
   Otherwise, the wm-fact's value is merely set to FALSE.
   @param ?id ID string of wm-fact
   @param ?enabled TRUE to enable retracting, FALSE to disable
  "
	(do-for-fact ((?wm wm-sync-map)) (eq ?wm:wm-fact-id ?id)
		(modify ?wm (wm-fact-retract ?enabled))
	)
)

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
				(printout error "Domain predicates may not contain argument value list " ?key crlf)
				(return FALSE)
			)
			(bind ?rv (append$ ?rv ?value))
			(bind ?l (+ ?l 2))
		)
	)
	(return ?rv)
)

(deffunction wm-sync-next-arg (?idx $?key)
	(bind ?L (length$ ?key))
	(if (= ?idx 0)
	 then
		(bind ?l (member$ args? ?key))
		(if ?l
		 then
			; +2: must have arg and value, then return +1, index for arg
			(if (<= (+ ?l 2) ?L)
			 then
				(if (eq (nth$ (+ ?l 2) ?key) [)
				 then
					(printout error "Domain facts may not contain argument value list " ?key crlf)
					(return -1)
				 else
					(return (+ ?l 1))
				)
			 else
				(return 0)
			)
		 else
			(return 0)
		)
	 else
		; +3: must have value and again arg and value, then return +2, index for next arg
		(if (<= (+ ?idx 3) ?L) then (return (+ ?idx 2)) else (return 0))
	)
)

(deffunction wm-sync-key-match (?key ?param-names ?param-values)
	"Check if the given key matches the passed arguments,"
	;(printout warn "Matching " ?key " to " ?param-names " " ?param-values crlf)
	(bind ?params-matched 0)
	(bind ?argidx (wm-sync-next-arg 0 ?key))
	(while (> ?argidx 0) do
		(bind ?arg (nth$ ?argidx ?key))
		(bind ?param-name-idx (member$ ?arg ?param-names))
		(if ?param-name-idx
		 then
			(if (neq (nth$ (+ ?argidx 1) ?key) (nth$ ?param-name-idx ?param-values)) then (return FALSE))
		 else
		 (return FALSE)
		)
		(bind ?argidx (wm-sync-next-arg ?argidx ?key))
		(bind ?params-matched (+ ?params-matched 1))
	)
	(if (or (< ?params-matched (length$ ?param-names)) (<> ?argidx 0))
	 then
		(return FALSE)
	 else
		(return TRUE)
	)
)

(defrule wm-sync-domain-fact-added
	(domain-predicate (name ?name) (param-names $?param-names))
	?df <- (domain-fact (name ?name) (param-values $?param-values))
	(not (domain-retracted-fact (name ?name) (param-values $?param-values)))
	(not (wm-sync-map (domain-fact-name ?name)
										(wm-fact-id ?id&:(eq ?id (wm-key-to-id domain fact ?name args?
																													 (domain-fact-key ?param-names ?param-values))))))
	=>
	(bind ?key (create$ domain fact ?name args? (domain-fact-key ?param-names ?param-values)))
	(bind ?wf (assert (wm-fact (key ?key) (type BOOL) (value TRUE))))
	(assert (wm-sync-map (wm-fact-id (wm-key-to-id ?key))
											 (wm-fact-key ?key)
											 (wm-fact-idx (fact-index ?wf))
											 (domain-fact-name ?name)
											 (domain-fact-idx (fact-index ?df))))
)

; (defrule wm-sync-domain-fact-true
; 	"A domain fact has been added for the first time."
; 	(wm-sync-map (wm-fact-id ?id) (wm-fact-idx 0) (domain-fact-name ?name) (domain-fact-idx ?idx))
; 	(domain-predicate (name ?name) (param-names $?param-names))
; 	?df <- (domain-fact (name ?name) (param-values $?param-values))
; 	(not (domain-retracted-fact (name ?name) (param-values $?param-values)))
; 	(test (eq ?idx (fact-index ?df)))
; 	(not (wm-fact (id ?id)))
; 	=>
; 	(assert (wm-fact (id ?id) (type BOOL) (value TRUE)))
; )

(defrule wm-sync-domain-fact-true
	"A domain fact has been added again, after being retracted before."
	?sf <- (wm-sync-map (wm-fact-id ?id) (domain-fact-name ?name) (domain-fact-idx ?idx))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key $?key) (value FALSE))
	?df <- (domain-fact (name ?name)
	                    (param-values $?param-values&:(wm-sync-key-match ?key ?param-names ?param-values)))
	(not (domain-retracted-fact (name ?name) (param-values $?param-values)))
	(test (< ?idx (fact-index ?df)))
	=>
	(modify ?wf (value TRUE))
	(modify ?sf (domain-fact-idx (fact-index ?df)))
)

(defrule wm-sync-domain-fact-removed
	"A domain fact has been removed, set wm fact to false or retract"
	?wm <- (wm-sync-map (wm-fact-id ?id) (domain-fact-name ?name) (domain-fact-idx ~0) (wm-fact-retract ?retract))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key $?key) (value TRUE))
	(not (domain-fact (name ?name)
	                  (param-values $?param-values&:(wm-sync-key-match ?key ?param-names ?param-values))))
	=>
	(if ?retract
	 then
		(retract ?wf)
		(modify ?wm (wm-fact-idx 0) (domain-fact-idx 0))
	 else
		(bind ?new-wf (modify ?wf (value FALSE)))
		(modify ?wm (wm-fact-idx (fact-index ?new-wf)) (domain-fact-idx 0))
	)
)

(defrule wm-sync-worldmodel-fact-added
	"A wm-fact has been added for the first time."
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key $?key&:(wm-key-prefix ?key domain fact ?name)))
	(not (wm-sync-map (domain-fact-name ?name)
										(wm-fact-id ?id&:(eq ?id (wm-key-to-id ?key)))))
	=>
	(bind ?df (assert (domain-fact (name ?name)
																 (param-values (wm-sync-key-arg-values ?key)))))
	(assert (wm-sync-map (domain-fact-name ?name)
											 (domain-fact-idx (fact-index ?df))
											 (wm-fact-id (wm-key-to-id ?key))
											 (wm-fact-key ?key)
											 (wm-fact-idx (fact-index ?wf))))
)

(defrule wm-sync-worldmodel-fact-true
	"The value of a wm fact is modified to TRUE."
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key $?key&:(wm-key-prefix ?key domain fact)) (type BOOL) (value TRUE))
	?wm <- (wm-sync-map (domain-fact-name ?name) (domain-fact-idx 0)
											(wm-fact-id ?id) (wm-fact-idx ?idx&:(< ?idx (fact-index ?wf))))
	=>
	(if (not (any-factp ((?df domain-fact))
											(and (eq ?df:name ?name)
													 (wm-sync-key-match ?key ?param-names ?df:param-values))))
	 then
		(bind ?df (assert (domain-fact (name ?name)
																	 (param-values (wm-sync-key-arg-values ?key)))))
		(modify ?wm (wm-fact-idx (fact-index ?wf)) (domain-fact-idx (fact-index ?df)))
	 else
		; If it was about to be retracted, don't...
		(do-for-fact ((?df domain-fact))
								 (and (eq ?df:name ?name)
											(wm-sync-key-match ?key ?param-names ?df:param-values))
			(delayed-do-for-all-facts ((?rf domain-retracted-fact))
																(and (eq ?rf:name ?name) (eq ?rf:param-values ?df:param-values))
				(retract ?rf)
			)
		)
		(modify ?wm (wm-fact-idx (fact-index ?wf)))
	)
)

(defrule wm-sync-worldmodel-fact-false
	"The value of a wm fact is modified to FALSE."
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key $?key&:(wm-key-prefix ?key domain fact)) (type BOOL) (value FALSE))
	?wm <- (wm-sync-map (domain-fact-name ?name) (domain-fact-idx ~0)
											(wm-fact-id ?id) (wm-fact-idx ?idx&:(< ?idx (fact-index ?wf))))
	(domain-fact (name ?name)
							 (param-values $?param-values&:(wm-sync-key-match ?key ?param-names ?param-values)))
	(not (domain-retracted-fact (name ?name) (param-values $?param-values)))
	=>
	(assert (domain-retracted-fact (name ?name)
																 (param-values (wm-sync-key-arg-values ?key))))
	(modify ?wm (wm-fact-idx (fact-index ?wf)) (domain-fact-idx 0))
)

(defrule wm-sync-worldmodel-fact-removed
	"The value of a wm fact has been removed."
	(domain-predicate (name ?name) (param-names $?param-names))
	?wm <- (wm-sync-map (domain-fact-name ?name) (domain-fact-idx ~0)
											(wm-fact-id ?id) (wm-fact-key $?key) (wm-fact-idx ~0))
	(domain-fact (name ?name)
							 (param-values $?param-values&:(wm-sync-key-match ?key ?param-names ?param-values)))
	(not (wm-fact (id ?id)))
	=>
	(assert (domain-retracted-fact (name ?name)
																 (param-values (wm-sync-key-arg-values ?key))))
	(modify ?wm (wm-fact-idx 0) (domain-fact-idx 0))
)

(defrule wm-sync-cleanup
	?wm <- (wm-sync-map (domain-fact-name ?name) (domain-fact-idx 0)
											(wm-fact-id ?id) (wm-fact-key $?key) (wm-fact-idx 0))
	(not (wm-fact (id ?id)))
	(domain-predicate (name ?name) (param-names $?param-names))
	(not (domain-fact (name ?name)
	                  (param-values $?param-values&:(wm-sync-key-match ?key ?param-names ?param-values))))
	=>
	(retract ?wm)
)
