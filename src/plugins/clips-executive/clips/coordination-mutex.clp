
;---------------------------------------------------------------------------
;  coordination-mutex.clp - CLIPS executive - asynchronous mutex handling
;
;  Created: Wed Apr 25 09:39:37 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Coordination through Mutual Exclusion (Locks)
; Mutexes are a principal to allow only a single entity access to a critical
; section. This specific implementation is meant to serve a group of robots
; for distributed locking. We do so by utilizing the robot-memory mutex
; capabilities.
;
; For each mutex, there exists a mutex fact. It is automatically create the
; first time a lock is requested. It may be created manually, or through
; mutex-create-async. Then, it will be added as soon as the update has
; propagated to the majority of the replica set.
;
; A lock may be requested through two ways:
; 1. call (mutex-try-lock-async ?name)
;    This is the preferred method. This will trigger acquisition of the lock
;    on the named mutex.
; 2. assert (or modify) a mutex fact of the form:
;    (mutex (name ?name) (request LOCK) (response NONE))
;
; Once a lock as been acquired, the response will be ACQUIRED, the state
; LOCKED, and locked-by will be the output of (cx-identity).
; A lock may be rejected, i.e., if there was a race and another entity
; acquired the lock first. Then, tje response will be REJECTED, the state
; LOCKED, and locked-by the name of the other entity.
;
; A lock may be released through two ways:
; 1. call (mutex-unlock-async ?name)
;    This is the preferred method. This will trigger asynchronous release of
;    the named mutex.
; 2. Modify the corresponding ?mf <- (mutex (name ?name)) fact like this:
;    (modify ?mf (request UNLOCK) (response NONE))
;
; Once the lock has been released, the lock will either be in state:
; - OPEN: lock has been released
; - LOCKED and locked-by an entity other than (cx-identity): the lock has
;   already been acquired by another entity.
;
; Mishandling is indicated through the "ERROR" response. The error-msg
; contains a user-readable error message.
;
; When handling mutex facts directly, you may ever only modify the request
; field, and the response field to set it to NONE (and NONE only). A request
; may NOT be aborted. If a request is PENDING no other request may be started
; or undefined behavior will occur.

(defglobal
	?*MUTEX-COLLECTION* = "robmem_coordination.mutex"
)

(deftemplate mutex
	(slot name (type SYMBOL))
	; state information is received from robot memory.
	; NEVER set this manually!
	(slot state (type SYMBOL) (allowed-values UNKNOWN OPEN LOCKED))
	(slot locked-by (type STRING))
	; request and response are set locally.
	; only ever set request from anywhere outside of this file. The response
	; will be set based on information from robot-memory.
	(slot request  (type SYMBOL) (allowed-values NONE LOCK UNLOCK CREATE))
	(slot response (type SYMBOL) (allowed-values NONE PENDING ACQUIRED REJECTED UNLOCKED ERROR))
	(slot error-msg (type STRING))
)

(deftemplate mutex-global-data
	(slot trigger-ptr (type EXTERNAL-ADDRESS))
)

; ***** FUNCTIONS ******

(deffunction mutex-try-lock-async (?name)
	(if (any-factp ((?m mutex)) (eq ?m:name ?name))
	then
		(do-for-fact ((?m mutex)) (eq ?m:name ?name)
			(if (eq ?m:state LOCKED)
			then
				; when a mutex is already locked, even if by ourself,
				; reject another request.
				; We may later make this configurable behavior.
				(modify ?m (response REJECTED) (error-msg (str-cat "Lock already held by " ?m:locked-by)))
			else
				(if (member$ ?m:request (create$ NONE UNLOCK))
				then
					(modify ?m (request LOCK) (response NONE) (error-msg ""))
				else
					(bind ?error-msg (str-cat "Mutex " ?name " already has pending LOCK request. "
					                          "This may lead to unpredictable behavior on both sides!"))
					(modify ?m (response ERROR) (error-msg ?error-msg))
				)
			)
		)
	else
		(assert (mutex (name ?name) (request LOCK)))
	)
)

(deffunction mutex-unlock-async (?name)
	(if (any-factp ((?m mutex)) (eq ?m:name ?name))
	then
		(do-for-fact ((?m mutex)) (eq ?m:name ?name)
			(if (and (eq ?m:state LOCKED) (neq ?m:locked-by (cx-identity)))
			then
				(modify ?m (response ERROR) (error-msg "Lock held by " ?m:locked-by ". Cannot release foreign lock."))
			else
				(if (member$ ?m:request (create$ NONE LOCK))
				then
					(modify ?m (request UNLOCK) (response NONE) (error-msg ""))
				else
					(bind ?error-msg (str-cat "Mutex " ?name " already has pending UNLOCK request. "
					                         "This may lead to unpredictable behavior on both sides!"))
					(modify ?m (response ERROR) (error-msg ?error-msg))
				)
			)
		)
	else
		(assert (mutex (name ?name) (request UNLOCK) (response UNLOCKED)))
	)
)

(deffunction mutex-create (?name)
	(robmem-mutex-create-async (str-cat ?name))
)


; ***** RULES ******

(defrule mutex-init
	(executive-init)
	(not (executive-finalize))
	(not (mutex-global-data))
	=>

	; Instead of using the defglobal, we could also read this from the config
	; (config-load "/plugins/robot-memory/coordination")
	
	(bind ?trigger-query (bson-create))
	(bind ?trigger-ptr (robmem-trigger-register ?*MUTEX-COLLECTION*
																							?trigger-query "mutex-trigger"))
	(bson-destroy ?trigger-query)

	(assert (mutex-global-data (trigger-ptr ?trigger-ptr)))
)

(defrule mutex-finalize
	(executive-finalize)
	?wi <- (mutex-global-data (trigger-ptr ?trigger-ptr))
	=>
	(robmem-trigger-destroy ?trigger-ptr)
	(retract ?wi)
)


(defrule mutex-print-error
	(mutex (name ?name) (request ?request) (response ERROR|REJECTED) (error-msg ?error-msg) (locked-by ?lb))
	=>
	(printout warn "Mutex " ?name " (request " ?request ", locked by '" ?lb "') reported an error: " ?error-msg crlf)
)

(defrule mutex-lock-start
	?mf <- (mutex (name ?name) (request LOCK) (response NONE) (state OPEN|UNKNOWN))
	=>
	(printout t "Requesting lock " ?name crlf)
	(robmem-mutex-try-lock-async (str-cat ?name) (cx-identity))
	(modify ?mf (response PENDING))
)

(defrule mutex-lock-invalid
	?mf <- (mutex (name ?name) (request LOCK) (response NONE)
								(state LOCKED) (locked-by ?lb))
	=>
	(modify ?mf (response ERROR) (error-msg (str-cat "Lock already locked by " ?lb)))
)

(defrule mutex-lock-acquired
	?mf <- (mutex (name ?name) (request LOCK) (response PENDING)
								(state LOCKED) (locked-by ?lb&:(eq ?lb (cx-identity))))
	=>
	(modify ?mf (response ACQUIRED))
)

(defrule mutex-lock-rejected
	?mf <- (mutex (name ?name) (request LOCK) (response PENDING)
								(state LOCKED) (locked-by ?lb&:(neq ?lb (cx-identity))))
	=>
	(modify ?mf (response REJECTED) (error-msg (str-cat "Lock held by " ?lb)))
)

(defrule mutex-lock-op-failed
	?mf <- (mutex (name ?name) (request LOCK) (response PENDING))
	?of <- (mutex-op-failed try-lock-async ?name)
	=>
	(retract ?of)
	(modify ?mf (response REJECTED) (error-msg "Lock held by unknown")
	            (state LOCKED) (locked-by "unknown"))
)

(defrule mutex-unlock-start
	?mf <- (mutex (name ?name) (request UNLOCK) (response NONE)
								(state LOCKED) (locked-by ?lb&:(eq ?lb (cx-identity))))
	=>
	(printout t "Requesting unlock " ?name crlf)
	(robmem-mutex-unlock-async (str-cat ?name) (cx-identity))
	(modify ?mf (response PENDING))
)

(defrule mutex-unlock-invalid
	?mf <- (mutex (name ?name) (request UNLOCK) (response NONE)
								(state LOCKED|UNKNOWN) (locked-by ?lb&:(neq ?lb (cx-identity))))
	=>
	(modify ?mf (response ERROR) (error-msg (str-cat "Locked by " ?lb " (and not by us)")))
)

(defrule mutex-unlock-succeeded
	?mf <- (mutex (name ?name) (request UNLOCK) (response PENDING)
								(state OPEN))
	=>
	(modify ?mf (response UNLOCKED))
)

(defrule mutex-unlock-succeeded-already-reacquired
	"Someone else already acquired the lock in the meantime."
	?mf <- (mutex (name ?name) (request LOCK) (response PENDING)
								(state LOCKED) (locked-by ?lb&:(neq ?lb (cx-identity))))
	=>
	(modify ?mf (response UNLOCKED))
)

(deffunction mutex-trigger-update (?obj)
	(bind ?id nil)
	(if (bson-has-field ?obj "o._id")
	then
		(bind ?id (sym-cat (bson-get ?obj "o._id")))
	else
		(bind ?id (sym-cat (bson-get ?obj "o2._id")))
	)	
	(if ?id then
		(bind ?locked FALSE)
		(bind ?locked-by "")

		(if (bson-has-field ?obj "o.$set")
		then
			; It's an incremental update
			(bind ?locked (bson-get ?obj "o.$set.locked"))
			(if (bson-has-field ?obj "o.$set.locked-by") then
				(bind ?locked-by (bson-get ?obj "o.$set.locked-by"))
				; else it's in $unset, lock is removed, leave empty string
			)
		else
			(bind ?locked (bson-get ?obj "o.locked"))
			(if (bson-has-field ?obj "o.locked-by") then
				(bind ?locked-by (bson-get ?obj "o.locked-by"))
			)
		)

		(if (any-factp ((?m mutex)) (eq ?m:name ?id))
		then
			(do-for-fact ((?m mutex)) (eq ?m:name ?id)
				(modify ?m (state (if ?locked then LOCKED else OPEN))
				           (locked-by ?locked-by))
			)
		else
			(assert (mutex (name ?id) (state (if ?locked then LOCKED else OPEN))
			               (locked-by ?locked-by)))
		)
	)
)

(deffunction mutex-trigger-delete (?obj)
	(bind ?id (sym-cat (bson-get ?obj "o._id")))
	(if ?id then
		(do-for-fact ((?m mutex)) (eq ?m:name ?id)
			(modify ?m (state OPEN) (locked-by ""))
		)
	)
)

(defrule mutex-trigger-event
	(wm-fact (key cx identity) (value ?identity))
	?rt <- (robmem-trigger (name "mutex-trigger") (ptr ?obj))
	=>
	(bind ?op (sym-cat (bson-get ?obj "op")))

	;(printout warn "Trigger: " (bson-tostring ?obj) crlf)
	(switch ?op
		(case i then (mutex-trigger-update ?obj))
		(case u then (mutex-trigger-update ?obj))
		(case d then (mutex-trigger-delete ?obj))
	)

	(bson-destroy ?obj)
	(retract ?rt)
)
