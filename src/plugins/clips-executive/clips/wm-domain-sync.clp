
;---------------------------------------------------------------------------
;  wm-domain-sync.clp - CLIPS executive - sync world and domain model
;
;  Created: Fri Dec 22 15:42:33 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate wm-sync-map-fact
	(slot wm-fact-id (type STRING))
	(multislot wm-fact-key (type SYMBOL))
	(slot wm-fact-idx (type INTEGER))
	(slot wm-fact-retract (type SYMBOL) (allowed-values TRUE FALSE) (default TRUE))
	(slot domain-fact-name (type SYMBOL))
	(slot domain-fact-idx (type INTEGER))
)

(deftemplate wm-sync-map-object-type
	(slot wm-fact-id (type STRING))
	(multislot wm-fact-key (type SYMBOL))
	(slot wm-fact-idx (type INTEGER))
	(slot domain-object-type (type SYMBOL))
)

(deftemplate wm-sync-remap-fact
	"Template to inform sync code about a path remapping for facts.

   The synchronization allows to remap domain-fact sync targets to
   arbitrary wm-fact IDs. This is useful to make the domain and world
   models more independent from each other, by using natural canonical
   naming for facts, e.g., mapping of /hardware/gripper/grasped in the
   world model to a holding predicate in the domain predicate.

   The condition for re-mapping is that the arguments are compatible,
   that is all arguments exist on both sides with the same names.
  "
	(slot domain-fact-name (type SYMBOL))
	(multislot wm-fact-key-path (type SYMBOL))
)

(deftemplate wm-sync-remap-object-type
	"Template to inform sync code about a path remapping for objects.

   The synchronization allows to remap domain-fact sync targets to
   arbitrary wm-fact IDs. This is useful to make the domain and world
   models more independent from each other, by using natural canonical
   naming for facts, e.g., mapping of /hardware/gripper/grasped in the
   world model to a holding predicate in the domain predicate.

   The condition for re-mapping is that the arguments are compatible,
   that is all arguments exist on both sides with the same names.
  "
	(slot domain-object-type (type SYMBOL))
	(multislot wm-fact-key (type SYMBOL))
)

(deffunction wm-sync-remap-fact-id-prefix ($?id-paths)
	"Add a number of simple namespace remappings.
   This function takes a number of IDs and re-maps them to domain
   facts taking the last part of the ID path as the predicate name.
   Example: (wm-sync-remap-fact-id-prefix '/wm/foo' '/wm/bar') will map
            the keys to the predicates foo and bar respectively.
   @param $?id-paths list of ID paths (without arguments), invalid
                     paths are ignored.
  "
	(foreach ?p ?id-paths
		(bind ?key (wm-id-to-key ?p))
		(bind ?path (wm-key-path ?key))
		(bind ?args (wm-key-args ?key))
		(if (> (length$ ?args) 0)
		 then
			(printout error "Cannot remap ID prefix with arguments (" ?p ")" crlf)
		 else
			(bind ?name (nth$ (length$ ?path) ?path))
			(assert (wm-sync-remap-fact (domain-fact-name ?name) (wm-fact-key-path ?key)))
		)
	)
)

(deffunction wm-sync-remap-object-id-prefix ($?id-paths)
	"Add a number of simple namespace remappings.
   This function takes a number of IDs and re-maps them to domain
   objects taking the last part of the ID path as the object type.
   Example: (wm-sync-remap-object-id-prefix '/wm/foo' '/wm/bar') will map
            the keys to the object types foo and bar respectively.
   @param $?id-paths list of ID paths (without arguments), invalid
                     paths are ignored.
  "
	(foreach ?p ?id-paths
		(bind ?key (wm-id-to-key ?p))
		(bind ?type (nth$ (length$ ?key) ?key))
		(assert (wm-sync-remap-object-type (domain-object-type ?type) (wm-fact-key ?key)))
	)
)

(deffunction wm-sync-set-wm-fact-retract (?id ?enabled)
	"Allow to set behavior of wm-fact removal on domain-fact removal.
   If wm-fact retraction is enabled (wm-fact-retract slot in wm-sync-map-fact TRUE),
   then a wm-fact is removed when the corresponding domain-fact is removed.
   Otherwise, the wm-fact's value is merely set to FALSE.
   @param ?id ID string of wm-fact
   @param ?enabled TRUE to enable retracting, FALSE to disable
  "
	(do-for-fact ((?wm wm-sync-map-fact)) (eq ?wm:wm-fact-id ?id)
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

(deffunction wm-sync-args-match (?key ?param-names ?param-values)
	"Check if the given key matches the passed arguments,"
	;(printout t "Matching " ?key " to " ?param-names " " ?param-values crlf)
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

(deffunction wm-sync-remapped-path-match (?key ?key-path)
	"Check if the given key matches the passed arguments,"
	(return (eq (wm-key-path ?key) ?key-path))
)

(deffunction wm-sync-remapped-pathargs-match (?key ?key-path ?param-names ?param-values)
	"Check if the given key matches the passed arguments,"
	(return (and (eq (wm-key-path ?key) ?key-path)
							 (wm-sync-args-match ?key ?param-names ?param-values)))
)

(defrule wm-sync-config-remap-fact-id-prefix
	(executive-init)
	(confval (path "/clips-executive/spec") (type STRING) (value ?spec))
  ?cf <- (confval (path ?path&:(eq (str-cat "/clips-executive/specs/" ?spec "/wm-remap/facts/id-prefix") ?path))
									(type STRING) (is-list TRUE) (list-value $?id-prefixes))
	=>
	(retract ?cf)
	(wm-sync-remap-fact-id-prefix ?id-prefixes)
)

(defrule wm-sync-config-remap-fact-name-id
	(executive-init)
	(confval (path "/clips-executive/spec") (type STRING) (value ?spec))
  ?cf <- (confval (path ?path&:(str-prefix (str-cat "/clips-executive/specs/" ?spec "/wm-remap/facts/name-id/") ?path))
									(type STRING) (value ?key-path) (is-list FALSE))
	=>
	(retract ?cf)
  (bind ?prefix (str-cat "/clips-executive/specs/" ?spec "/wm-remap/facts/name-id/"))
  (bind ?name (sym-cat (sub-string (+ (str-length ?prefix) 1) (str-length ?path) ?path)))
	(assert (wm-sync-remap-fact (domain-fact-name ?name) (wm-fact-key-path (wm-id-to-key ?key-path))))
)

(defrule wm-sync-config-remap-object-id-prefix
	(executive-init)
	(confval (path "/clips-executive/spec") (type STRING) (value ?spec))
  ?cf <- (confval (path ?path&:(eq (str-cat "/clips-executive/specs/" ?spec "/wm-remap/objects/id-prefix") ?path))
									(type STRING) (is-list TRUE) (list-value $?id-prefixes))
	=>
	(retract ?cf)
	(wm-sync-remap-object-id-prefix ?id-prefixes)
)

(defrule wm-sync-config-remap-object-name-id
	(executive-init)
	(confval (path "/clips-executive/spec") (type STRING) (value ?spec))
  ?cf <- (confval (path ?path&:(str-prefix (str-cat "/clips-executive/specs/" ?spec "/wm-remap/objects/name-id/") ?path))
									(type STRING) (value ?id) (is-list FALSE))
	=>
	(retract ?cf)
  (bind ?prefix (str-cat "/clips-executive/specs/" ?spec "/wm-remap/objects/name-id/"))
  (bind ?type (sym-cat (sub-string (+ (str-length ?prefix) 1) (str-length ?path) ?path)))
	(assert (wm-sync-remap-object-type (domain-object-type ?type) (wm-fact-key (wm-id-to-key ?id))))
)

(defrule wm-sync-domain-fact-added
	"For a recently added domain fact, add a wm-fact."
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	(domain-predicate (name ?name) (param-names $?param-names))
	?df <- (domain-fact (name ?name) (param-values $?param-values))
	(not (wm-sync-remap-fact (domain-fact-name ?name)))
	(not (wm-sync-map-fact (domain-fact-name ?name)
												 (wm-fact-id ?id&:(eq ?id (wm-key-to-id domain fact
																																(domain-fact-key ?name ?param-names ?param-values))))))
	=>
	(bind ?key (create$ domain fact (domain-fact-key ?name ?param-names ?param-values)))
	(bind ?wf (assert (wm-fact (key ?key) (type BOOL) (value TRUE))))
	(assert (wm-sync-map-fact (wm-fact-id (wm-key-to-id ?key))
														(wm-fact-key ?key)
														(wm-fact-idx (fact-index ?wf))
														(domain-fact-name ?name)
														(domain-fact-idx (fact-index ?df))))
)

(defrule wm-sync-domain-fact-remapped-added
	"For a recently added domain fact, add a path-remapped wm-fact."
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	(domain-predicate (name ?name) (param-names $?param-names))
	?df <- (domain-fact (name ?name) (param-values $?param-values))
	(wm-sync-remap-fact (domain-fact-name ?name) (wm-fact-key-path $?key-path))
	(not (wm-sync-map-fact (domain-fact-name ?name)
												 (wm-fact-key $?key&:(wm-sync-remapped-pathargs-match ?key ?key-path ?param-names ?param-values))))
	=>
	(bind ?key (create$ ?key-path (domain-fact-args ?param-names ?param-values)))
	(bind ?wf (assert (wm-fact (key ?key) (type BOOL) (value TRUE))))
	(assert (wm-sync-map-fact (wm-fact-id (wm-key-to-id ?key))
														(wm-fact-key ?key)
														(wm-fact-idx (fact-index ?wf))
														(domain-fact-name ?name)
														(domain-fact-idx (fact-index ?df))))
)

; (defrule wm-sync-domain-fact-true
; 	"A domain fact has been added for the first time."
; 	(wm-sync-map-fact (wm-fact-id ?id) (wm-fact-idx 0) (domain-fact-name ?name) (domain-fact-idx ?idx))
; 	(domain-predicate (name ?name) (param-names $?param-names))
; 	?df <- (domain-fact (name ?name) (param-values $?param-values))
; 	(test (eq ?idx (fact-index ?df)))
; 	(not (wm-fact (id ?id)))
; 	=>
; 	(assert (wm-fact (id ?id) (type BOOL) (value TRUE)))
; )

(defrule wm-sync-domain-fact-true
	"A domain fact has been added again, after being retracted before."
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	?sf <- (wm-sync-map-fact (wm-fact-id ?id) (domain-fact-name ?name) (domain-fact-idx ?idx))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key $?key) (value FALSE))
	?df <- (domain-fact (name ?name)
	                    (param-values $?param-values&:(wm-sync-args-match ?key ?param-names ?param-values)))
	(test (< ?idx (fact-index ?df)))
	=>
	(modify ?wf (value TRUE))
	(modify ?sf (domain-fact-idx (fact-index ?df)))
)

(defrule wm-sync-domain-fact-removed
	"A domain fact has been removed, set wm fact to false or retract"
  (declare (salience ?*SALIENCE-WM-SYNC-DEL*))
	?wm <- (wm-sync-map-fact (wm-fact-id ?id) (domain-fact-name ?name) (domain-fact-idx ~0)
													 (wm-fact-retract ?retract))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key $?key) (value TRUE))
	(not (domain-fact (name ?name)
	                  (param-values $?param-values&:(wm-sync-args-match ?key ?param-names ?param-values))))
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
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	(domain-predicate (name ?name) (param-names $?param-names))
	(not (wm-sync-remap-fact (domain-fact-name ?name)))
	?wf <- (wm-fact (id ?id) (key $?key&:(wm-key-prefix ?key domain fact ?name)))
	(not (wm-sync-map-fact (domain-fact-name ?name)
												 (wm-fact-id ?id&:(eq ?id (wm-key-to-id ?key)))))
	=>
	(bind ?df (assert (domain-fact (name ?name)
																 (param-values (wm-sync-key-arg-values ?key)))))
	(assert (wm-sync-map-fact (domain-fact-name ?name)
														(domain-fact-idx (fact-index ?df))
														(wm-fact-id (wm-key-to-id ?key))
														(wm-fact-key ?key)
														(wm-fact-idx (fact-index ?wf))))
)

(defrule wm-sync-worldmodel-fact-remapped-added
	"A wm-fact has been added for the first time."
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	(domain-predicate (name ?name) (param-names $?param-names))
	(wm-sync-remap-fact (domain-fact-name ?name) (wm-fact-key-path $?key-path))
	?wf <- (wm-fact (id ?id) (key $?key&:(wm-sync-remapped-path-match ?key ?key-path)))
	(not (wm-sync-map-fact (domain-fact-name ?name)
												 (wm-fact-key $?key&:(wm-sync-remapped-pathargs-match ?key ?key-path ?param-names (wm-sync-key-arg-values ?key)))))
	=>
	(bind ?df (assert (domain-fact (name ?name)
																 (param-values (wm-sync-key-arg-values ?key)))))
	(assert (wm-sync-map-fact (domain-fact-name ?name)
														(domain-fact-idx (fact-index ?df))
														(wm-fact-id (wm-key-to-id ?key))
														(wm-fact-key ?key)
														(wm-fact-idx (fact-index ?wf))))
)

(defrule wm-sync-worldmodel-fact-true
	"The value of a wm fact is modified to TRUE."
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key $?key) (type BOOL) (value TRUE))
	?wm <- (wm-sync-map-fact (domain-fact-name ?name) (domain-fact-idx 0)
													 (wm-fact-id ?id) (wm-fact-idx ?idx&:(< ?idx (fact-index ?wf))))
	=>
	(if (not (any-factp ((?df domain-fact))
											(and (eq ?df:name ?name)
													 (wm-sync-args-match ?key ?param-names ?df:param-values))))
	 then
		; The domain fact does not exist, create it
		(bind ?df (assert (domain-fact (name ?name)
																	 (param-values (wm-sync-key-arg-values ?key)))))
		(modify ?wm (wm-fact-idx (fact-index ?wf)) (domain-fact-idx (fact-index ?df)))
	 else
		(modify ?wm (wm-fact-idx (fact-index ?wf)))
	)
)

(defrule wm-sync-worldmodel-fact-false
	"The value of a wm fact is modified to FALSE."
  (declare (salience ?*SALIENCE-WM-SYNC-DEL*))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wf <- (wm-fact (id ?id) (key $?key) (type BOOL) (value FALSE))
	?wm <- (wm-sync-map-fact (domain-fact-name ?name) (domain-fact-idx ~0)
													 (wm-fact-id ?id) (wm-fact-idx ?idx&:(< ?idx (fact-index ?wf))))
	?df <- (domain-fact (name ?name)
											(param-values $?param-values&:(wm-sync-args-match ?key ?param-names ?param-values)))
	=>
	(retract ?df)
	(modify ?wm (wm-fact-idx (fact-index ?wf)) (domain-fact-idx 0))
)

(defrule wm-sync-worldmodel-fact-removed
	"The value of a wm fact has been removed."
  (declare (salience ?*SALIENCE-WM-SYNC-DEL*))
	(domain-predicate (name ?name) (param-names $?param-names))
	?wm <- (wm-sync-map-fact (domain-fact-name ?name) (domain-fact-idx ~0)
													 (wm-fact-id ?id) (wm-fact-key $?key) (wm-fact-idx ~0))
	?df <- (domain-fact (name ?name)
											(param-values $?param-values&:(wm-sync-args-match ?key ?param-names ?param-values)))
	(not (wm-fact (id ?id)))
	=>
	(retract ?df)
	(modify ?wm (wm-fact-idx 0) (domain-fact-idx 0))
)

(defrule wm-sync-fact-cleanup
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	?wm <- (wm-sync-map-fact (domain-fact-name ?name) (domain-fact-idx 0)
													 (wm-fact-id ?id) (wm-fact-key $?key) (wm-fact-idx 0))
	(not (wm-fact (id ?id)))
	(domain-predicate (name ?name) (param-names $?param-names))
	(not (domain-fact (name ?name)
	                  (param-values $?param-values&:(wm-sync-args-match ?key ?param-names ?param-values))))
	=>
	(retract ?wm)
)

(defrule wm-sync-domain-object-type-added
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	(domain-object-type (name ?type-name))
	(not (wm-sync-map-object-type (domain-object-type ?type-name)))
	(not (wm-sync-remap-object-type (domain-object-type ?type-name)))
	=>
	(bind ?key (create$ domain objects-by-type ?type-name))
	(assert (wm-sync-map-object-type (wm-fact-id (wm-key-to-id ?key)) (wm-fact-key ?key)
																	 (wm-fact-idx 0) (domain-object-type ?type-name)))
)

(defrule wm-sync-domain-object-type-remapped-added
  (declare (salience ?*SALIENCE-WM-SYNC-DEL*))
	(domain-object-type (name ?type-name))
	(not (wm-sync-map-object-type (domain-object-type ?type-name)))
	(wm-sync-remap-object-type (domain-object-type ?type-name) (wm-fact-key $?key))
	=>
	(assert (wm-sync-map-object-type (wm-fact-id (wm-key-to-id ?key)) (wm-fact-key ?key)
																	 (wm-fact-idx 0) (domain-object-type ?type-name)))
)

(defrule wm-sync-domain-object-mapping-added
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	?wm <- (wm-sync-map-object-type (wm-fact-id ?id) (wm-fact-key $?key) (wm-fact-idx ?idx)
																	(domain-object-type ?type))
	(not (wm-fact (key $?key) (type SYMBOL) (is-list TRUE)))
	=>
	(bind ?wf (assert (wm-fact (id ?id) (key ?key) (type SYMBOL) (is-list TRUE) (values))))
	; If the fact idx is zero, or no domain-object of the given type exists, then
	; this is the first time the mapping is added. Update the fact index as we do
	; not expect any previous information.
	; However, if the idx is not zero and there are domain-object of the respective
	; type, do not (yet) update the wm-fact-idx since this means that the wm-fact
  ; was retracted altogether, meaning that any still existing domain-object that
	; were mentioned should now be deleted by the object-removed rule.
	(if (or (= ?idx 0) (not (any-factp ((?df domain-object)) (eq ?df:type ?type))))
		then (modify ?wm (wm-fact-idx (fact-index ?wf)))
	)
)

(defrule wm-sync-domain-object-added
	"For a recently added domain objects, add a wm-fact."
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	(domain-object (name ?name) (type ?type))
	?wm <- (wm-sync-map-object-type (wm-fact-id ?id) (wm-fact-idx ?wf-idx)
																	(domain-object-type ?type))
	?wf <- (wm-fact (id ?id) (type SYMBOL) (is-list TRUE) (values $?objs&~:(member$ ?name ?objs)))
	(test (<= (fact-index ?wf) ?wf-idx))
	=>
	(bind ?new-wf (modify ?wf (values (append$ ?objs ?name))))
	(modify ?wm (wm-fact-idx (fact-index ?new-wf)))
)

(defrule wm-sync-domain-object-removed
  (declare (salience ?*SALIENCE-WM-SYNC-DEL*))
	?wm <- (wm-sync-map-object-type (wm-fact-id ?id) (wm-fact-idx ?wf-idx)
																	(domain-object-type ?type))
	?wf <- (wm-fact (id ?id) (type SYMBOL) (is-list TRUE) (values $? ?name $?))
	(not (domain-object (name ?name) (type ?type)))
	(test (<= (fact-index ?wf) ?wf-idx))
	=>
	(bind ?objs (fact-slot-value ?wf values))
	(bind ?obj-idx (member$ ?name ?objs))
	(bind ?new-objs (delete$ ?objs ?obj-idx ?obj-idx))
	(bind ?new-wf (modify ?wf (values ?new-objs)))
	(modify ?wm (wm-fact-idx (fact-index ?new-wf)))
)

(defrule wm-sync-worldmodel-object-added
  (declare (salience ?*SALIENCE-WM-SYNC-ADD*))
	?wm <- (wm-sync-map-object-type (wm-fact-id ?id) (wm-fact-idx ?wf-idx) (domain-object-type ?type))
	?wf <- (wm-fact (id ?id) (type SYMBOL) (is-list TRUE) (values $? ?name $?))
	(not (domain-object (name ?name) (type ?type)))
	(test (> (fact-index ?wf) ?wf-idx))
	=>
	; While the rule checks if there is any object that is missing, the update might
	; actually have added multiple objects with one update. Hence, check them all.
	(bind ?objs (fact-slot-value ?wf values))
	(foreach ?o ?objs
		(if (not (any-factp ((?df domain-object))	(and (eq ?df:name ?o) (eq ?df:type ?type))))
		 then
			(assert (domain-object (name ?o) (type ?type)))

		)
	)
	(modify ?wm (wm-fact-idx (fact-index ?wf)))
)

(defrule wm-sync-worldmodel-object-removed
  (declare (salience ?*SALIENCE-WM-SYNC-DEL*))
	?wm <- (wm-sync-map-object-type (wm-fact-id ?id) (wm-fact-idx ?wf-idx) (domain-object-type ?type))
	(domain-object (name ?name) (type ?type))
	?wf <- (wm-fact (id ?id) (type SYMBOL) (is-list TRUE) (values $?objs&~:(member$ ?name ?objs)))
	(test (> (fact-index ?wf) ?wf-idx))
	=>
	(delayed-do-for-all-facts ((?df domain-object)) (not (member$ ?df:name ?objs))
		(retract ?df)
	)
	(modify ?wm (wm-fact-idx (fact-index ?wf)))
)
