
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
	(multislot lock-time (type INTEGER) (cardinality 2 2))
	; request and response are set locally.
	; only ever set request from anywhere outside of this file. The response
	; will be set based on information from robot-memory.
	(slot request  (type SYMBOL) (allowed-values NONE LOCK UNLOCK CREATE FLUSH-LOCKS RENEW-LOCK))
	(multislot pending-requests  (type SYMBOL) (allowed-values NONE UNLOCK AUTO-RENEW-PROC))
	(slot response (type SYMBOL) (allowed-values NONE PENDING ACQUIRED REJECTED UNLOCKED ERROR))
	(slot auto-renew (type SYMBOL) (allowed-values TRUE FALSE))
	(slot error-msg (type STRING))
)

(deftemplate mutex-expire-task
	(slot task (type SYMBOL) (allowed-values FLUSH EXPIRE))
	(slot state (type SYMBOL) (allowed-values NONE PENDING COMPLETED FAILED))
	(slot max-age-sec (type FLOAT))
)

(deftemplate mutex-global-data
	(slot trigger-ptr (type EXTERNAL-ADDRESS))
)

; ***** FUNCTIONS ******

(deffunction mutex-try-lock-async (?name $?args)
	(bind ?auto-renew TRUE)
	(if (> (length$ ?args) 0)	then
		(bind ?auto-renew (nth$ 1 ?args))
	)

	(if (any-factp ((?m mutex)) (eq ?m:name ?name))
	then
		(do-for-fact ((?m mutex)) (eq ?m:name ?name)
			(if (eq ?m:state LOCKED)
			then
				; when a mutex is already locked, even if by ourself,
				; reject another request.
				; We may later make this configurable behavior.
				(modify ?m (request LOCK) (response REJECTED)
				           (error-msg (str-cat "Lock already held by " ?m:locked-by)))
			else
				(if (eq ?m:request NONE)
				then
					(modify ?m (request LOCK) (response NONE) (auto-renew ?auto-renew) (error-msg ""))
				else
					(bind ?error-msg (str-cat "Mutex " ?name " already has pending " ?m:request " request. "
					                          "This may lead to unpredictable behavior on both sides!"))
					(modify ?m (request LOCK) (response ERROR) (error-msg ?error-msg))
				)
			)
		)
	else
		(assert (mutex (name ?name) (request LOCK) (auto-renew ?auto-renew)))
	)
)

(deffunction mutex-unlock-async (?name)
	(if (any-factp ((?m mutex)) (eq ?m:name ?name))
	then
		(do-for-fact ((?m mutex)) (eq ?m:name ?name)
			(if (and (eq ?m:state LOCKED) (neq ?m:locked-by (cx-identity)))
			then
			(modify ?m (request UNLOCK) (response ERROR)
			           (error-msg (str-cat "Lock held by " ?m:locked-by ". Cannot release foreign lock.")))
			else
				(if (and (eq ?m:request RENEW-LOCK) (member$ AUTO-RENEW-PROC ?m:pending-requests))
				then ; auto-renew is running, wait for this to finish and then unlock
					(modify ?m (pending-requests ?m:pending-requests UNLOCK))
				else
					(if (eq ?m:request NONE)
					then
						(modify ?m (request UNLOCK) (response NONE) (error-msg ""))
					else
						(bind ?error-msg (str-cat "Mutex " ?name " already has pending UNLOCK request. "
						                         "This may lead to unpredictable behavior on both sides!"))
						(modify ?m (request UNLOCK) (response ERROR) (error-msg ?error-msg))
					)
				)
			)
		)
	else
		(assert (mutex (name ?name) (request UNLOCK) (response UNLOCKED)))
	)
)

(deffunction mutex-renew-lock-async (?name)
	(do-for-fact ((?m mutex)) (eq ?m:name ?name)
		(if (neq ?m:state LOCKED)
		then
			(modify ?m (request RENEW-LOCK) (response ERROR)
			           (error-msg (str-cat "Mutex " ?name " not LOCKED. Cannot renew.")))
		else
			(if (neq ?m:locked-by (cx-identity))
			then
				(modify ?m (request RENEW-LOCK) (response ERROR)
				           (error-msg (str-cat "Lock held by " ?m:locked-by ". Cannot renew foreign lock.")))
			else
				(if (neq ?m:request NONE)
				then
					(bind ?error-msg (str-cat "Mutex " ?name " already has pending " ?m:request " request. "
				                            "This may lead to unpredictable behavior on both sides!"))
					(modify ?m (request RENEW-LOCK) (response ERROR) (error-msg ?error-msg))
				else
					(modify ?m (request RENEW-LOCK) (response NONE) (error-msg ""))
				)
			)
		)
	)
)

(deffunction mutex-flush-locks-async ()
	(if (any-factp ((?m mutex-expire-task)) TRUE)
	then
		(printout error "Mutex lock expiration already running" crlf)
	else
		(assert (mutex-expire-task (task FLUSH) (state NONE) (max-age-sec 0.0)))
	)
)

(deffunction mutex-expire-locks-async ()
	(if (any-factp ((?m mutex-expire-task)) TRUE)
	then
		(printout error "Mutex lock expiration already running" crlf)
	else
		(do-for-fact ((?wf wm-fact)) (eq ?wf:id "/config/coordination/mutex/max-age-sec")
			(assert (mutex-expire-task (task EXPIRE) (state NONE) (max-age-sec ?wf:value)))
		)
	)
)

(deffunction mutex-create (?name)
	(robmem-mutex-create-async (str-cat ?name))
)


(deffunction mutex-register-trigger ()
	; Instead of using the defglobal, we could also read this from the config
	; (config-load "/plugins/robot-memory/coordination")
	
	(bind ?trigger-query (bson-create))
	(bind ?trigger-ptr (robmem-trigger-register ?*MUTEX-COLLECTION*
																							?trigger-query "mutex-trigger"))
	(bson-destroy ?trigger-query)
)

; ***** RULES ******


(defrule mutex-init
	(executive-init)
	(not (executive-finalize))
	(not (mutex-global-data))
	=>
	(mutex-register-trigger)
)

(defrule mutex-init-register-trigger-done
	?fb <- (mutex-trigger-register-feedback SUCCESS "mutex-trigger" ?trigger-ptr)
	=>
	(retract ?fb)
	(printout info "Registered trigger 'mutex-trigger'" crlf)
	(assert (mutex-global-data (trigger-ptr ?trigger-ptr)))
)

(defrule mutex-init-register-trigger-failed
	?fb <- (mutex-trigger-register-feedback FAIL "mutex-trigger")
	=>
	(retract ?fb)
	(printout warn "Failed to register trigger 'mutex-trigger', retrying" crlf)
	(mutex-register-trigger)
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
	(mutex-global-data)
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
	(modify ?mf (response ERROR) (error-msg (str-cat "Mutex already locked by " ?lb)))
)

(defrule mutex-lock-rejected
	?mf <- (mutex (name ?name) (request LOCK) (response PENDING)
								(state LOCKED) (locked-by ?lb&:(neq ?lb (cx-identity))))
	=>
	(modify ?mf (response REJECTED) (error-msg (str-cat "Lock held by " ?lb)))
)

(defrule mutex-lock-op-failed
	?mf <- (mutex (name ?name) (request LOCK) (response PENDING))
	?of <- (mutex-op-feedback try-lock-async FAIL ?name)
	=>
	(retract ?of)
	(modify ?mf (response REJECTED) (error-msg "Lock held by 'unknown'")
	            (state LOCKED) (locked-by "unknown"))
)

(defrule mutex-lock-op-cleanup
	(not (mutex (name ?name) (response PENDING)))
	?of <- (mutex-op-feedback try-lock-async FAIL ?name)
	=>
	(retract ?of)
)

(defrule mutex-renew-lock-start
	(mutex-global-data)
	?mf <- (mutex (name ?name) (request RENEW-LOCK) (response NONE)
								(state LOCKED) (locked-by ?lb&:(eq ?lb (cx-identity))))
	=>
	(printout debug "Renewing lock " ?name crlf)
	(robmem-mutex-renew-lock-async (str-cat ?name) (cx-identity))
	(modify ?mf (response PENDING))
)

(defrule mutex-renew-lock-invalid-not-locked
	?mf <- (mutex (name ?name) (request RENEW-LOCK) (response NONE) (state OPEN))
	=>
	(modify ?mf (response ERROR) (error-msg (str-cat "Mutex is not locked")))
)

(defrule mutex-renew-lock-invalid-foreign-lock
	?mf <- (mutex (name ?name) (request RENEW-LOCK) (response NONE) (state OPEN)
								(locked-by ?lb&:(neq ?lb (cx-identity))))
	=>
	(modify ?mf (response ERROR) (error-msg (str-cat "Mutex locked by " ?lb " (not '" (cx-identity) "')")))
)

(defrule mutex-renew-lock-op-failed
	?mf <- (mutex (name ?name) (request RENEW-LOCK) (response PENDING))
	?of <- (mutex-op-feedback renew-lock-async FAIL ?name)
	=>
	(retract ?of)
	(modify ?mf (response REJECTED) (error-msg "Renewing the lock failed"))
)

(defrule mutex-unlock-start
	(mutex-global-data)
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

(defrule mutex-unlock-succeeded-already-reacquired
	"Someone else already acquired the lock in the meantime."
	?mf <- (mutex (name ?name) (request UNLOCK) (response PENDING)
	              (state LOCKED) (locked-by ?lb&:(neq ?lb (cx-identity))))
	=>
	(modify ?mf (response UNLOCKED))
)

(defrule mutex-expire-locks-start
	(mutex-global-data)
	?mf <- (mutex-expire-task (task ?task) (state NONE) (max-age-sec ?max-age-sec))
	=>
	(printout debug ?task " locks " crlf)
	(robmem-mutex-expire-locks-async ?max-age-sec)
	(modify ?mf (state PENDING))
)

(defrule mutex-expire-locks-succeeded
	?mf <- (mutex-expire-task (task ?task) (state PENDING))
	?of <- (mutex-op-feedback expire-locks-async OK)
	=>
	(retract ?of)
	(modify ?mf (state COMPLETED))
)

(defrule mutex-expire-locks-failed
	?mf <- (mutex-expire-task (task ?task) (state PENDING))
	?of <- (mutex-op-feedback expire-locks-async FAIL)
	=>
	(retract ?of)
	(modify ?mf (state FAILED))
)

(defrule mutex-lock-auto-renew
	(time $?now)
	(wm-fact (id "/config/coordination/mutex/renew-interval") (type FLOAT|UINT|INT) (value ?renew-interval))
	?mf <- (mutex (name ?name) (state LOCKED) (request NONE) (pending-requests)
								(locked-by ?lb&:(eq ?lb (cx-identity)))
								(lock-time $?lt&:(timeout (time-trunc-ms (now-systime)) ?lt ?renew-interval)))
	=>
	(printout debug "Automatic renewal of lock for mutex " ?name crlf)
	(modify ?mf (request RENEW-LOCK) (response NONE) (pending-requests AUTO-RENEW-PROC))
)

(defrule mutex-lock-auto-renew-done
	?mf <- (mutex (name ?name) (state LOCKED)
								(request RENEW-LOCK) (response ACQUIRED)
								(pending-requests AUTO-RENEW-PROC $?pending-requests))
	=>
	(printout debug "Automatic renewal of lock for mutex " ?name " completed" crlf)
	(modify ?mf (request NONE) (response NONE) (pending-requests ?pending-requests))
)

(defrule mutex-lock-auto-renew-lock-open
	?mf <- (mutex (name ?name) (state OPEN)
	              (request RENEW-LOCK)
	              (pending-requests AUTO-RENEW-PROC $?pending-requests))
  =>
  (printout error "Received response to renewal request for " ?name
                  " but lock has expired in the meantime. " crlf)
  (modify ?mf (request NONE) (response NONE) (pending-requests ?pending-requests))
)

(defrule mutex-lock-auto-renew-locked-by-other-entity
  ?mf <- (mutex (name ?name) (state LOCKED) (request RENEW-LOCK)
                (pending-requests AUTO-RENEW-PROC $?pending-requests)
                (locked-by ?lb&:(neq ?lb (cx-identity))))
  =>
  (printout error "Received response to renewal request for " ?name
                  " but lock is locked by " ?lb ". " crlf)
  (modify ?mf (request NONE) (response NONE) (pending-requests ?pending-requests))
)

(defrule mutex-lock-auto-renew-failed
	?mf <- (mutex (name ?name) (state LOCKED)
								(request RENEW-LOCK) (response REJECTED) (locked-by ?lb)
								(pending-requests AUTO-RENEW-PROC $?pending-requests))
	=>
	(printout error "Renewing lock for mutex " ?name " failed. LOST LOCK!" crlf)
	(bind ?new-lb (if (eq ?lb (cx-identity)) then "lost" else ?lb))
	(modify ?mf (request NONE) (response NONE) (pending-requests ?pending-requests) (locked-by ?new-lb))
)

(defrule mutex-run-pending-request
	?mf <- (mutex (name ?name) (request NONE) (pending-requests ?request $?pending-requests))
	=>
	(printout t "Run pending request for mutex " ?name ": " ?request crlf)
	(modify ?mf (request ?request) (response NONE) (pending-requests ?pending-requests))
)

(deffunction mutex-trigger-update (?obj)
	(bind ?doc (bson-get ?obj "fullDocument"))
	(bind ?id (sym-cat (bson-get ?doc "_id")))
	(if ?id then
		(bind ?locked FALSE)
		(bind ?locked-by "")
		(bind ?lock-time (create$ 0 0))

		(bind ?locked (bson-get ?doc "locked"))
		(if (bson-has-field ?doc "locked-by") then
			(bind ?locked-by (bson-get ?doc "locked-by"))
		)
		(if (bson-has-field ?doc "lock-time") then
			(bind ?lock-time (bson-get-time ?doc "lock-time"))
		)

		(do-for-fact ((?m mutex)) (eq ?m:name ?id)
      (if ?locked then
        (if (eq ?locked-by (cx-identity)) then
          (if (or (eq ?m:request LOCK) (eq ?m:request RENEW-LOCK)) then
            ; Mutex is locked by us and we requested it, all good.
            (modify ?m (response ACQUIRED))
           else
             (if (eq ?m:state OPEN) then
               ; Mutex is newly locked by us but we never requested it.
               (printout error "Acquired a mutex without a request,"
                               " releasing the mutex again!" crlf)
               (mutex-unlock-async ?id)
             )
          )
        )
       else ; Mutex is unlocked.
        (if (eq ?m:request UNLOCK) then
          (modify ?m (response UNLOCKED))
        )
      )
    )
		(if (any-factp ((?m mutex)) (eq ?m:name ?id))
		then
			(do-for-fact ((?m mutex)) (eq ?m:name ?id)
				(modify ?m (state (if ?locked then LOCKED else OPEN))
				           (locked-by ?locked-by) (lock-time ?lock-time))
			)
		else
			(assert (mutex (name ?id) (state (if ?locked then LOCKED else OPEN))
			               (locked-by ?locked-by) (lock-time ?lock-time)))
		)
	)
)

(deffunction mutex-trigger-delete (?obj)
	(bind ?id (sym-cat (bson-get ?obj "fullDocument._id")))
	(if ?id then
		(do-for-fact ((?m mutex)) (eq ?m:name ?id)
			(modify ?m (state OPEN) (locked-by ""))
		)
	)
)

(defrule mutex-trigger-event
	(wm-fact (key cx identity) (value ?identity))
	?rt <- (robmem-trigger (name "mutex-trigger") (ptr ?obj) (rcvd-at $?recv-time))
	(not (robmem-trigger (name "mutex-trigger") (rcvd-at $?comp-time&:(time> ?recv-time ?comp-time))))
	=>
	(bind ?op (sym-cat (bson-get ?obj "operationType")))

	(printout debug "Trigger: " (bson-tostring ?obj) crlf)
	(switch ?op
		(case insert then (mutex-trigger-update ?obj))
		(case update then (mutex-trigger-update ?obj))
		(case delete then (mutex-trigger-delete ?obj))
	)

	(bson-destroy ?obj)
	(retract ?rt)
)
