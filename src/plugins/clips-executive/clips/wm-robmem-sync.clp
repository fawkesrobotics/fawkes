
;---------------------------------------------------------------------------
;  wm-robmem-sync.clp - CLIPS executive - sync wm through robot memory
;
;  Created: Fri Apr 13 19:28:17 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Worldmodel Sync via Robot Memory
; This code is meant to synchronize the world model of multiple robots
; through the Robot Memory, i.e., a replicated MongoDB database where each
; robot has a replica member.
;
; Worldmodel -> Robot Memory
; The synchronization must be enabled through configuring a number of
; wm-fact ID/key prefixes through wm-robmem-sync-conf facts. A convenience
; function wm-robmem-sync-enable for a variable number of IDs is provided
; to simplify configuration.
;
; For each such prefix, the fact base is monitored for wm-fact facts. If a
; new fact is observed, a wm-robmem-sync-map-entry is created. It stores
; the wm-fact ID, key, and fact index, as well as an update timestamp.
; The fact index is used to detect updates issues through, e.g., (modify).
; The key is used to match the key prefix of the opt-in configuration.
; The ID is used to store and reference entries in the database.
;
; From the perspective of this sync, we have a single replicated databse
; with a single collection "worldmodel". For each synchronized wm-fact one
; entry is added to the database, mirroring the wm-fact data.
;
; When storing information in the database, we use an atomic upsert
; constrained by a unique index for conflict resolution:
; - a unique index on the wm-fact ID ensures that there is always only at
;   most a single entry for a wm-fact ID.
; - the update query has as a condition that an existing entry in the
;   database has a smaller update-timestamp
; - if no entry exists, the entry is inserted (upsert). However, the query
;   can also match if there is only newer information in the database. The
;   then resulting insert will fail since the unique index prevents a
;   second entry from being added.
; For now, we use a simple update. Later, we may want to switch to
; find_one_and_update to have more control and verify the result.
; This requires a precise and close synchronization of all replica set
; participants! Webview 2.0 enables monitoring this.
;
; So far, queries are provided to provide this basic direction.
;
;
; Robot Memory -> Worldmodel
; This direction is handled through triggers (later: change streams). That
; is, we register the oplog and receive notification for all modificatons
; to the database. We can use the timestamp again to verify that the data
; is more recent than our current information. We can directly reflect
; updates to the database in our worldmodel iff we have a matching prefix
; configuration.
;
; This has not been fully implemented, yet.
;
;
; Future considerations:
; - we could compare also values and not just the fact-idx in order to
;   recognize modify with the same values (i.e., no-op modifies).
; - we could keep individual shadow copies of the worldmodel and have
;   explicit conflict resolution code.
; - we could store the full world model in a single document. This
;   would give better consistency guarantees but incur tremendous
;   (bandwidth) overhead.
; - asyncrhonous operations
; - sharding
; - locking


(defglobal
	?*WM-ROBMEM-SYNC-COLLECTION* = "syncedrobmem.worldmodel"
)

(deftemplate wm-robmem-sync-map-entry
	(slot wm-fact-id  (type STRING))
	(multislot wm-fact-key (type SYMBOL))
	(slot wm-fact-idx (type INTEGER))
	(multislot update-timestamp (type INTEGER) (cardinality 2 2))
)

(deftemplate wm-robmem-sync-conf
	(multislot wm-fact-key-prefix (type SYMBOL))
	(slot enabled (type SYMBOL) (allowed-values TRUE FALSE))
)

(deffunction wm-robmem-sync-enable ($?id-prefixes)
	(foreach ?id-prefix ?id-prefixes
		(assert (wm-robmem-sync-conf (wm-fact-key-prefix (wm-id-to-key ?id-prefix))))
	)
)

(defrule wm-robmem-sync-init
	(not (wm-robmem-sync-initialized))
	=>
	(assert (wm-robmem-sync-initialized))
	(bind ?keys (bson-create))
	(bson-append ?keys "id" 1)
	(robmem-create-unique-index ?*WM-ROBMEM-SYNC-COLLECTION* ?keys)
	(bson-destroy ?keys)
)


(deffunction wm-robmem-sync-create-fact-doc (?wf ?update-timestamp)
	(bind ?doc (bson-create))
	(bson-append-time ?doc "update-timestamp" ?update-timestamp)
	(bson-append ?doc "id" (fact-slot-value ?wf id))

	(bson-append ?doc "type" (fact-slot-value ?wf type))
	(bson-append ?doc "is-list" (fact-slot-value ?wf is-list))
	(if (fact-slot-value ?wf is-list)
	 then
		(bson-append-array ?doc "values" (fact-slot-value ?wf values))
	 else
		(bson-append ?doc "value" (fact-slot-value ?wf value))
	)
	(return ?doc)
)

(deffunction wm-robmem-sync-create-query (?id ?update-timestamp)
	(bind ?query (bson-create))
	(bind ?query-lte-timestamp (bson-create))
	(bson-append-time ?query-lte-timestamp "$lte" ?update-timestamp)
	(bson-append ?query "id" ?id)
	(bson-append ?query "update-timestamp" ?query-lte-timestamp)
	(return ?query)
)

(deffunction wm-robmem-sync-fact-update (?wf ?update-timestamp)
	(bind ?doc (wm-robmem-sync-create-fact-doc ?wf ?update-timestamp))
	(bind ?query (wm-robmem-sync-create-query (fact-slot-value ?wf id) ?update-timestamp))

	;(printout t "Query: " (bson-tostring ?query) crlf)
	;(printout t "Doc: " (bson-tostring ?doc) crlf)

	(robmem-upsert ?*WM-ROBMEM-SYNC-COLLECTION* ?doc ?query)
)

(defrule wm-robmem-sync-fact-added
	(wm-robmem-sync-conf (wm-fact-key-prefix $?key-prefix) (enabled TRUE))
	?wf <- (wm-fact (id ?id) (key $?key-prefix $?rest)
									(type ?type) (is-list ?is-list) (value ?value) (values $?values))
	(not (wm-robmem-sync-map-entry (wm-fact-id ?id)))
	=>
	;(printout error "Add " ?id " to robot memory" crlf)
	(bind ?now (now))
	(assert (wm-robmem-sync-map-entry (wm-fact-id ?id) (wm-fact-key ?key-prefix ?rest)
																		(wm-fact-idx (fact-index ?wf))
																		(update-timestamp ?now)))
	
	(wm-robmem-sync-fact-update ?wf ?now)
)

(defrule wm-robmem-sync-fact-removed
	(wm-robmem-sync-conf (wm-fact-key-prefix $?key-prefix) (enabled TRUE))
	?sm <- (wm-robmem-sync-map-entry (wm-fact-id ?id) (wm-fact-key $?key-prefix $?rest)
																	 (update-timestamp $?update-timestamp))
	(not (wm-fact (id ?id)))
	=>
	;(printout error "Remove " ?id " from robot memory" crlf)
	(retract ?sm)

	(bind ?query (wm-robmem-sync-create-query ?id ?update-timestamp))

	(robmem-remove ?*WM-ROBMEM-SYNC-COLLECTION* ?query)
)

(defrule wm-robmem-sync-fact-modified
	(wm-robmem-sync-conf (wm-fact-key-prefix $?key-prefix) (enabled TRUE))
	?wf <- (wm-fact (id ?id) (key $?key-prefix $?rest))
	?sm <- (wm-robmem-sync-map-entry (wm-fact-id ?id)
																	 (wm-fact-idx ?idx&:(neq ?idx (fact-index ?wf))))
	=>
	;(printout error "Modify " ?id " in robot memory" crlf)
	(bind ?now (now))
	(modify ?sm (wm-fact-idx (fact-index ?wf)) (update-timestamp ?now))

	(wm-robmem-sync-fact-update ?wf ?now)
)
