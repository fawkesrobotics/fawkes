
;---------------------------------------------------------------------------
;  cedar-ros.clp - CEDAR system monitoring for ROS
;
;  Created: Fri Sep 30 13:47:41 2011
;  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
;             2011-2013  RWTH Aachen University (KBSG)
;             2011       SRI International
;             2011       Carnegie Mellon University
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

;(defmodule ROS-HEALTH
;  (export deftemplate ?ALL)
;)

; --- DEFINE GLOBALS

(defglobal ;ROS-HEALTH
  ?*ROS-COLLECT-PERIOD* = 1.0
  ?*NODE-TIMEOUT*       = 5.0
  ?*CONNECTION-TIMEOUT* = 3.0
)

(deffacts ros-facts
  (system-condition (id ROS))
  (timer (name ros-collect))
)

; --- TEMPLATES
(deftemplate model-ros-topic
  (slot name)
  (slot type)
  (slot importance
	(type SYMBOL) (default CRITICAL)
	(allowed-values CRITICAL RECOMMENDED IGNORED))
)

(deftemplate model-ros-node
  (slot name (type STRING))
  (multislot published)
  (multislot subscribed)
  (multislot services)
  (slot importance
        (type SYMBOL) (default CRITICAL)
        (allowed-values CRITICAL RECOMMENDED IGNORED))
  ; nodemon: node supports node monitoring, use for heartbeat detection
  ; single-threaded: not is single-threaded and if a node is unreachable this
  ;          might just indicate, for example, a long-running service call
  (multislot flags (type SYMBOL) (allowed-values nodemon single-threaded))
)


(deftemplate monitor-ros-node
  (slot name (type STRING))
  (slot state (type SYMBOL)
        (allowed-values IGNORED ALIVE UNREACHABLE DEAD) (default ALIVE))
  (multislot last-seen (type INTEGER) (cardinality 2 2) (default-dynamic (now)))
  (multislot unreachable-since
             (type INTEGER) (cardinality 2 2) (default-dynamic (now)))
  (multislot missing-published (type STRING))
  (multislot missing-subscribed (type STRING))
  (multislot missing-services (type STRING))
)

(deftemplate model-ros-topic-connection
  (slot topic (type STRING))
  (slot from (type STRING))
  (slot to (type STRING))
  (slot importance
        (type SYMBOL) (default CRITICAL)
        (allowed-values CRITICAL RECOMMENDED IGNORED))
)

(deftemplate monitor-ros-topic-connection
  (slot topic (type STRING))
  (slot from (type STRING))
  (slot to (type STRING))
  (slot state (type SYMBOL) (allowed-values UP DOWN LIMBO))
  (multislot limbo-since (type INTEGER) (cardinality 2 2) (default-dynamic (now)))
)

; --- FUNCTIONS

(deffunction ros-node> (?a ?b)
  (> (str-compare (fact-slot-value ?a name) (fact-slot-value ?b name)) 0))


(deffunction ros-topic-conn-fact> (?a ?b)
  (bind ?a:topic (fact-slot-value ?a topic))
  (bind ?b:topic (fact-slot-value ?b topic))
  (bind ?a:from (fact-slot-value ?a from))
  (bind ?b:from (fact-slot-value ?b from))
  (bind ?a:to (fact-slot-value ?a to))
  (bind ?b:to (fact-slot-value ?b to))
  (or (> (str-compare ?a:topic ?b:topic) 0)
      (and (eq ?a:topic ?b:topic)
           (or (> (str-compare ?a:from ?b:from) 0)
               (and (eq ?a:from ?b:from) (> (str-compare ?a:to ?b:to) 0))))
  )
)

; --- RULES - initialization
;(defrule ros-health-init
;  (initial-fact)
;  =>
;)


; --- RULES - data collection and cleanup
(defrule ros-collect
  (time $?now)
  ?tf <- (timer (name ros-collect) (time $?t&:(timeout ?now ?t ?*ROS-COLLECT-PERIOD*)))
  =>
  (modify ?tf (time ?now))
  ;(printout t "Requesting ROS nodes" crlf)
  (ros-get-nodes)
  (ros-get-topic-connections)
  (assert (ros-collect-process ?now))
)

(defrule ros-collect-cleanup
  (declare (salience -10000))
  ?rcpf <- (ros-collect-process $?)
  =>
  (retract ?rcpf)
  (ros-cleanup)
)


; --- RULES - nodes
(defrule ros-node-alive-start
  (ros-collect-process $?now)
  (ros-node (name ?n))
  (model-ros-node (name ?n) (importance ~IGNORED))
  (not (monitor-ros-node (name ?n)))
  =>
  (printout t "Node " ?n " is ALIVE" crlf)
  (assert (monitor-ros-node (name ?n)))
)

(defrule ros-node-dead-start
  (ros-collect-process $?now)
  (model-ros-node (name ?n) (importance ~IGNORED))
  (not (ros-node (name ?n)))
  (not (monitor-ros-node (name ?n)))
  =>
  (printout t "Node " ?n " is DEAD (missing on start)" crlf)
  (assert (monitor-ros-node (name ?n) (state DEAD)))
)

(defrule ros-node-unreachable
  (ros-collect-process $?now)
  (model-ros-node (name ?n) (importance ~IGNORED))
  (not (ros-node (name ?n)))
  ?mn <- (monitor-ros-node (name ?n) (state ALIVE))
  =>
  (printout t "Node " ?n " is UNREACHABLE" crlf)
  (modify ?mn (state UNREACHABLE) (unreachable-since ?now))
)

(defrule ros-node-alive-recover
  (ros-collect-process $?now)
  (ros-node (name ?n))
  (model-ros-node (name ?n) (importance ~IGNORED))
  ?mf <- (monitor-ros-node (name ?n) (state ~ALIVE))
  =>
  (printout t "Node " ?n " is ALIVE (recovered)" crlf)
  (modify ?mf (state ALIVE))
)

(defrule ros-node-alive-missing
  (ros-collect-process $?now)
  (model-ros-node (name ?n) (importance ~IGNORED)
		  (published $?mpub) (subscribed $?msub) (services $?msrv))
  (ros-node (name ?n) (published $?pub) (subscribed $?sub) (services $?srv))
  ; Match monitored ROS node, match all missing-* slots already here, doing that
  ; only in the (or ...) constraint would fail our printouts due to undeclared variables 
  ?mf <- (monitor-ros-node (name ?n) (state ALIVE)
	   (missing-published $?mp) (missing-subscribed $?ms) (missing-services $?mr))
  ; Mind the connective constraints: this is just the shortest way of matching the
  ; missing-* fields matched in the condition before
  (or (monitor-ros-node (name ?n&:(neq ?mp (set-diff ?mpub ?pub))))
      (monitor-ros-node (name ?n&:(neq ?ms (set-diff ?msub ?sub))))
      (monitor-ros-node (name ?n&:(neq ?mr (set-diff ?msrv ?srv)))))
  =>
  ; Update our missing something info
  (bind ?missing-published  (set-diff ?mpub ?pub))
  (bind ?missing-subscribed (set-diff ?msub ?sub))
  (bind ?missing-services   (set-diff ?msrv ?srv))
  (modify ?mf (missing-published ?missing-published)
	      (missing-subscribed ?missing-subscribed)
	      (missing-services ?missing-services))
  (if (> (length$ ?missing-published) 0)
   then
    (printout t "Node " ?n " is missing published topics: " ?missing-published crlf)
   else
    (if (> (length$ ?mp) 0) then
      (printout t "Node " ?n " is now publishing all expected topics " crlf))
  )
  (if (> (length$ ?missing-subscribed) 0)
   then
    (printout t "Node " ?n " is missing subscribed topics: " ?missing-subscribed crlf)
   else
    (if (> (length$ ?ms) 0) then
      (printout t "Node " ?n " is now subscribing all expected topics " crlf))
  )
  (if (> (length$ ?missing-services) 0)
   then
    (printout t "Node " ?n " is missing services: " ?missing-services crlf)
   else
    (if (> (length$ ?mr) 0) then
      (printout t "Node " ?n " is now advertising all expected services " crlf))
  )
)

(defrule ros-node-dead
  (ros-collect-process $?now)
  (model-ros-node (name ?n) (importance ~IGNORED))
  ?mn <- (monitor-ros-node (name ?n) (state UNREACHABLE)
	   (unreachable-since $?ut&:(timeout ?now ?ut ?*NODE-TIMEOUT*)))
  =>
  (printout t "Node " ?n " is DEAD" crlf)
  (modify ?mn (state DEAD))
)

(defrule ros-node-ignored-model
  (ros-collect-process $?now)
  (ros-node (name ?n))
  (model-ros-node (name ?n) (importance IGNORED))
  (not (monitor-ros-node (name ?n)))
  =>
  (printout t "Node " ?n " will be IGNORED (as per model)" crlf)
  (assert (monitor-ros-node (name ?n) (state IGNORED)))
)

(defrule ros-node-ignored-unknown
  (ros-collect-process $?now)
  (ros-node (name ?n))
  (not (model-ros-node (name ?)))
  (not (monitor-ros-node (name ?n)))
  =>
  (printout t "Node " ?n " will be IGNORED (not in model)" crlf)
  (assert (monitor-ros-node (name ?n) (state IGNORED)))
)


; --- RULES - connections

(defrule ros-connection-up
  (ros-collect-process $?now)
  (ros-topic-connection (topic ?topic) (from ?from) (to ?to))
  (model-ros-topic-connection (topic ?topic) (from ?from) (to ?to) (importance ~IGNORED))
  (not (monitor-ros-topic-connection (topic ?topic) (from ?from) (to ?to)))
  =>
  (printout t "Connection for " ?topic ": " ?from " -> " ?to " is UP" crlf)
  (assert (monitor-ros-topic-connection (topic ?topic) (from ?from) (to ?to) (state UP)))
)

(defrule ros-connection-down-start
  (ros-collect-process $?now)
  (model-ros-topic-connection (topic ?topic) (from ?from) (to ?to) (importance ~IGNORED))
  (not (ros-topic-connection (topic ?topic) (from ?from) (to ?to)))
  (not (monitor-ros-topic-connection (topic ?topic) (from ?from) (to ?to)))
  =>
  (printout t "Connection for " ?topic ": " ?from " -> " ?to " is DOWN" crlf)
  (assert (monitor-ros-topic-connection (topic ?topic) (from ?from) (to ?to) (state DOWN)))
)

(defrule ros-connection-up-recover
  (ros-collect-process $?now)
  (ros-topic-connection (topic ?topic) (from ?from) (to ?to))
  (model-ros-topic-connection (topic ?topic) (from ?from) (to ?to) (importance ~IGNORED))
  ?mf <- (monitor-ros-topic-connection (topic ?topic) (from ?from) (to ?to) (state ~UP))
  =>
  (printout t "Connection for " ?topic ": " ?from " -> " ?to " is UP (recovered)" crlf)
  (modify ?mf (state UP))
)

(defrule ros-connection-limbo
  (ros-collect-process $?now)
  (model-ros-topic-connection (topic ?topic) (from ?from) (to ?to) (importance ~IGNORED))
  (not (ros-topic-connection (topic ?topic) (from ?from) (to ?to)))
  ?mf <- (monitor-ros-topic-connection (topic ?topic) (from ?from) (to ?to)
				       (state ~LIMBO&~DOWN))
  =>
  (printout t "Connection for " ?topic ": " ?from " -> " ?to " is in LIMBO" crlf)
  (modify ?mf (state LIMBO) (limbo-since ?now))
)

(defrule ros-connection-down
  (ros-collect-process $?now)
  (model-ros-topic-connection (topic ?topic) (from ?from) (to ?to) (importance ~IGNORED))
  (not (ros-topic-connection (topic ?topic) (from ?from) (to ?to)))
  ?mf <- (monitor-ros-topic-connection (topic ?topic) (from ?from) (to ?to) (state LIMBO)
           (limbo-since $?lt&:(timeout ?now ?lt ?*CONNECTION-TIMEOUT*)))
  =>
  (printout t "Connection for " ?topic ": " ?from " -> " ?to " is DOWN" crlf)
  (modify ?mf (state DOWN))
)



; --- RULES - system state
(defrule ros-syscond
  (ros-collect-process $?now)
  ?cf <- (system-condition (id ROS) (updated $?u&:(neq ?u ?now)))
  =>
  (bind ?cond GREEN)
  (bind ?desc (create$))

  ; Note that the ordering of the tests is important, first check for
  ; yellow conditions, and only then for red.

  ; YELLOW conditions

  ; Unreachable nodes (transient error) cause our state to become yellow
  (do-for-all-facts ((?node monitor-ros-node)) (eq ?node:state UNREACHABLE)
    (bind ?desc (append$ ?desc (str-cat "Node " ?node:name " is unreachable")))
    (bind ?cond YELLOW)
  )

  ; If a node is not pub/sub/adv what it is supposed to go to yellow alert
  (do-for-all-facts ((?node monitor-ros-node) (?model model-ros-node))
    ; check for critical nodes with missing topics or services
    (and (eq ?node:name ?model:name) (eq ?model:importance CRITICAL)
	 (or (> (length$ ?node:missing-published) 0)
	     (> (length$ ?node:missing-subscribed) 0)
	     (> (length$ ?node:missing-services) 0)))
    (if (> (length$ ?node:missing-published) 0) then
      (bind ?desc (append$ ?desc (str-cat "Node " ?node:name " missing published topics: "
					  (implode$ ?node:missing-published)))))
    (if (> (length$ ?node:missing-subscribed) 0) then
      (bind ?desc (append$ ?desc (str-cat "Node " ?node:name " missing subscribed topics: "
					  (implode$ ?node:missing-subscribed)))))
    (if (> (length$ ?node:missing-services) 0) then
      (bind ?desc (append$ ?desc (str-cat "Node " ?node:name " missing services: "
					  (implode$ ?node:missing-services)))))
    (bind ?cond YELLOW)
  )

  ; Required connection is in limbo
  (do-for-all-facts ((?conn monitor-ros-topic-connection)) (eq ?conn:state LIMBO)
    (bind ?desc (append$ ?desc (str-cat "Connection for " ?conn:topic ": "
					?conn:from " -> " ?conn:to " is in limbo")))
    (bind ?cond YELLOW)
  )

  ; RED conditions

  ; Required connection is down
  (do-for-all-facts ((?conn monitor-ros-topic-connection)) (eq ?conn:state DOWN)
    (bind ?desc (append$ ?desc (str-cat "Connection for " ?conn:topic ": "
					?conn:from " -> " ?conn:to " is down")))
    (bind ?cond RED)
  )

  ; Node is dead
  (do-for-all-facts ((?node monitor-ros-node)) (eq ?node:state DEAD)
    (bind ?desc (append$ ?desc (str-cat "Node " ?node:name " is dead")))
    (bind ?cond RED)
  )

  (modify ?cf (cond ?cond) (desc (syscond-join-desc ?desc)) (updated ?now))
)
