
;---------------------------------------------------------------------------
;  robmem-wait-init.clp - wait for robot-memory to be fully up
;
;  Created: Tue Apr 23 10:02:16 2018
;  Copyright  2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(blackboard-open-reading "MongoDBManagedReplicaSetInterface" "robot-memory-distributed")
(blackboard-open-reading "MongoDBManagedReplicaSetInterface" "robot-memory-local")

(defrule robmem-wait-init-distributed-ok
	(MongoDBManagedReplicaSetInterface (id "robot-memory-distributed")
	                                   (member_status PRIMARY|SECONDARY)
	                                   (primary_status HAVE_PRIMARY))
	(not (executive-init-signal (id robmem-initialized-distributed)))
	(not (executive-init-signal (id robmem-initialized)))
	=>
	(printout t "Replica set robot-memory-distributed is up" crlf)
	(assert (executive-init-signal (id robmem-initialized-distributed)))
)

(defrule robmem-wait-init-local-ok
	(MongoDBManagedReplicaSetInterface (id "robot-memory-local")
	                                   (member_status PRIMARY|SECONDARY)
	                                   (primary_status HAVE_PRIMARY))
	(not (executive-init-signal (id robmem-initialized-local)))
	(not (executive-init-signal (id robmem-initialized)))
	=>
	(printout t "Replica set robot-memory-local is up" crlf)
	(assert (executive-init-signal (id robmem-initialized-local)))
)

(defrule robmem-wait-init-all-ok
	?s1 <- (executive-init-signal (id robmem-initialized-local))
	?s2 <- (executive-init-signal (id robmem-initialized-distributed))
	(not (executive-init-signal (id robmem-initialized)))
	=>
	(retract ?s1 ?s2)
	(printout t "Both replica sets are up" crlf)
	(assert (executive-init-signal (id robmem-initialized)))
)
