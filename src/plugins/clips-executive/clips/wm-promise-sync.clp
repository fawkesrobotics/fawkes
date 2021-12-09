;---------------------------------------------------------------------------
;  wm-promise-sync.clp - CLIPS executive - sync world and domain promise model
;
;  Created: Wed Dec 8
;  Copyright  2021  Tarik Viehmann
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction wm-promise-sync-wm-key
		(?goal ?name ?args ?promising-agent ?valid-at ?negated ?dni)
	" Create a wm-fact key from the information stored in a promise fact"
	(return (create$
		domain promise ?name ?args from ?goal
		args? promising-agent ?promising-agent
		      valid-at ?valid-at
		      negated ?negated
		      do-not-invalidate ?dni
	))
)

(deftemplate wm-promise-sync-map
	(slot wm-fact-id (type STRING))
	(multislot wm-fact-key (type SYMBOL))
	(slot wm-fact-idx (type INTEGER))
	(slot wm-fact-retract (type SYMBOL) (allowed-values TRUE FALSE) (default TRUE))
	(slot domain-promise-idx (type INTEGER))
)

(deffunction wm-promise-sync-set-wm-fact-retract (?id ?enabled)
	"Allow to set behavior of wm-fact removal on domain-promise removal.
	 If wm-fact retraction is enabled (wm-fact-retract slot in
	 wm-promise-sync-map TRUE),
	 then a wm-fact is removed when the corresponding domain-promise is removed.
	 Otherwise, the wm-fact's value is merely set to FALSE.
	 @param ?id ID string of wm-fact
	 @param ?enabled TRUE to enable retracting, FALSE to disable
	"
	(do-for-fact ((?wm wm-promise-sync-map)) (eq ?wm:wm-fact-id ?id)
		(modify ?wm (wm-fact-retract ?enabled))
	)
)

(defrule wm-promise-sync-domain-promise-added
	"For a recently added domain promise, add a wm-fact."
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	?df <- (domain-promise (name ?name) (param-values $?promise-args)
	                       (promising-goal ?goal)
	                       (promising-agent ?promising-agent)
	                       (valid-at ?va-int)
	                       (negated ?negated)
	                       (do-not-invalidate ?dni)
	                       (active TRUE)
	       )
	(not (wm-promise-sync-map (wm-fact-key ?key&:(eq ?key
	          (wm-promise-sync-wm-key ?goal ?name ?promise-args
	                                  ?promising-agent (sym-cat ?va-int) ?negated ?dni)
	      )
	)))
	=>
	(bind ?key (wm-promise-sync-wm-key ?goal ?name ?promise-args
	                                      ?promising-agent (sym-cat ?va-int) ?negated ?dni)
	)
	(bind ?wf (assert (wm-fact (key ?key) (type BOOL) (value TRUE))))

	(assert (wm-promise-sync-map (wm-fact-id (wm-key-to-id ?key))
	                             (wm-fact-key ?key)
	                             (wm-fact-idx (fact-index ?wf))
	                             (domain-promise-idx (fact-index ?df))))
)

(defrule wm-promise-sync-domain-promise-true
	"A domain promise has been added again, after being retracted before."
	(declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	?sf <- (wm-promise-sync-map (wm-fact-id ?id) (wm-fact-key $?key)
	                            (domain-promise-idx ?idx))
	?wf <- (wm-fact (id ?id) (key domain promise ?name $?promise-args from ?goal
	                          args? promising-agent ?promising-agent
	                                valid-at ?valid-at
	                                negated ?negated
	                                do-not-invalidate ?dni)
	                (value FALSE))
	?df <- (domain-promise (name ?name) (param-values $?promise-args)
	                       (promising-goal ?goal)
	                       (promising-agent ?promising-agent)
	                       (valid-at ?va-int&:(= ?va-int (string-to-field (str-cat ?valid-at))))
	                       (negated ?negated)
	                       (do-not-invalidate ?dni)
	                       (active TRUE)
	       )
	(test (< ?idx (fact-index ?df)))
	=>
	(modify ?wf (value TRUE))
	(modify ?sf (domain-promise-idx (fact-index ?df)))
)

(defrule wm-promise-sync-domain-promise-removed
	"A domain promise has been removed, set wm fact to false or retract"
  (declare (salience ?*SALIENCE-WM-SYNC-DEL*))
	?wm <- (wm-promise-sync-map (wm-fact-id ?id) (domain-promise-idx ~0)
	                            (wm-fact-retract ?retract))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key domain promise ?name $?promise-args from ?goal
	                          args? promising-agent ?promising-agent
	                                valid-at ?valid-at
	                                negated ?negated
	                                do-not-invalidate ?dni)
	                (value TRUE))
	(not (domain-promise (name ?name) (param-values $?promise-args)
	                     (promising-goal ?goal)
	                     (promising-agent ?promising-agent)
	                     (valid-at ?va-int&:(= ?va-int (string-to-field (str-cat ?valid-at))))
	                     (negated ?negated)
	                     (do-not-invalidate ?dni)
	                     (active TRUE)
	     )
	)
	=>
	(if ?retract
	 then
		(retract ?wf)
		(modify ?wm (wm-fact-idx 0) (domain-promise-idx 0))
	 else
		(bind ?new-wf (modify ?wf (value FALSE)))
		(modify ?wm (wm-fact-idx (fact-index ?new-wf)) (domain-promise-idx 0))
	)
)

(defrule wm-promise-sync-worldmodel-fact-added
	"A wm-fact for a promise has been added without existing domain promise."
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key domain promise ?name $?promise-args from ?goal
	                          args? promising-agent ?promising-agent
	                                valid-at ?valid-at
	                                negated ?negated
	                                do-not-invalidate ?dni)
	       )
	(not (wm-promise-sync-map (wm-fact-id ?id)))
	=>
	(bind ?df (assert
		(domain-promise (name ?name) (param-values $?promise-args)
		                (promising-goal ?goal) (promising-agent ?promising-agent)
		                (valid-at (string-to-field (str-cat ?valid-at))) (negated ?negated)
		                (do-not-invalidate ?dni)
		                (active TRUE)
		)
	))
	(assert
		(wm-promise-sync-map (domain-promise-idx (fact-index ?df))
		                     (wm-fact-id ?id)
		                     (wm-fact-key (fact-slot-value ?wf key))
		                     (wm-fact-idx (fact-index ?wf))
		)
	)
)

(defrule wm-promise-sync-worldmodel-fact-true
	"The value of a wm fact is modified to TRUE."
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key domain promise ?name $?promise-args from ?goal
	                          args? promising-agent ?promising-agent
	                                valid-at ?valid-at
	                                negated ?negated
	                                do-not-invalidate ?dni)
	                (type BOOL) (value TRUE)
	       )
	?wm <- (wm-promise-sync-map (domain-promise-idx 0)
	                            (wm-fact-id ?id)
	                            (wm-fact-idx ?idx&:(< ?idx (fact-index ?wf))))
	=>
	(if (not (any-factp ((?df domain-promise))
	                    (and (eq ?df:name ?name)
	                         (eq ?df:promising-goal ?goal)
	                         (eq ?df:param-values ?promise-args)
	                    )
	         )
	    )
	 then
		; The domain fact does not exist, create it
		(bind ?df (assert
			(domain-promise (name ?name) (param-values $?promise-args)
			                (promising-goal ?goal) (promising-agent ?promising-agent)
			                (valid-at (string-to-field (str-cat ?valid-at))) (negated ?negated)
			                (do-not-invalidate ?dni)
			                (active TRUE)
			)
		))
		(modify ?wm (wm-fact-idx (fact-index ?wf))
		            (domain-promise-idx (fact-index ?df)))
	 else
		(modify ?wm (wm-fact-idx (fact-index ?wf)))
	)
)

(defrule wm-promise-sync-worldmodel-fact-false
	"The value of a wm fact is modified to FALSE."
	(declare (salience ?*SALIENCE-WM-SYNC-DEL*))
	?wf <- (wm-fact (id ?id) (key domain promise ?name $?promise-args from ?goal
	                          args? promising-agent ?promising-agent
	                                valid-at ?valid-at
	                                negated ?negated
	                                do-not-invalidate ?dni)
	                (type BOOL) (value FALSE)
	       )
	?wm <- (wm-promise-sync-map (domain-promise-idx ~0) (wm-fact-id ?id)
	                            (wm-fact-idx ?idx&:(< ?idx (fact-index ?wf))))
	?df <- (domain-promise (name ?name) (param-values $?promise-args)
	                       (promising-goal ?goal)
	                       (promising-agent ?promising-agent)
	                       (valid-at ?va-int&:(= ?va-int (string-to-field (str-cat ?valid-at))))
	                       (negated ?negated)
	                       (do-not-invalidate ?dni)
	                       (active TRUE)
	       )
	=>
	(retract ?df)
	(modify ?wm (wm-fact-idx (fact-index ?wf)) (domain-promise-idx 0))
)

(defrule wm-promise-sync-worldmodel-fact-removed
	"The value of a wm fact has been removed."
	(declare (salience ?*SALIENCE-WM-SYNC-DEL*))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wm <- (wm-promise-sync-map (domain-promise-idx ~0) (wm-fact-id ?id)
	                            (wm-fact-key $?key) (wm-fact-idx ~0))
	?df <- (domain-promise (name ?name) (param-values $?promise-args)
	                       (promising-goal ?goal)
	                       (promising-agent ?promising-agent)
	                       (valid-at ?valid-at) (negated ?negated)
	                       (do-not-invalidate ?dni)
	                       (active TRUE)
	       )
	(test (eq ?key
	          (wm-promise-sync-wm-key ?goal ?name ?promise-args
	                                     ?promising-agent (sym-cat ?valid-at) ?negated ?dni)
	      )
	)
	(not (wm-fact (id ?id)))
	=>
	(retract ?df)
	(modify ?wm (wm-fact-idx 0) (domain-promise-idx 0))
)

(defrule wm-promise-sync-fact-cleanup
	(declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	?wm <- (wm-promise-sync-map (domain-promise-idx 0) (wm-fact-id ?id)
	         (wm-fact-key domain promise ?name $?promise-args from ?goal
	                      args? promising-agent ?promising-agent
	                            valid-at ?valid-at
	                            negated ?negated
	                            do-not-invalidate ?dni)
	         (wm-fact-idx 0))
	(not (wm-fact (id ?id)))
	(not (domain-promise (name ?name) (param-values $?promise-args)
	                       (promising-goal ?goal)
	                       (promising-agent ?promising-agent)
	                       (valid-at ?va-int&:(= ?va-int (string-to-field (str-cat ?valid-at))))
	                       (negated ?negated)
	                       (do-not-invalidate ?dni)
	                       (active TRUE)
	      )
	)
	=>
	(retract ?wm)
)
