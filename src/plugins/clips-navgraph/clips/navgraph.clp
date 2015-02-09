
;---------------------------------------------------------------------------
;  navgraph.clp - CLIPS navgraph access
;
;  Created: Sat Jun 22 02:21:44 2013
;  Copyright  2012-2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Examples:
; 1. Find the node closest to a position ?pos (with extended comment)
;    ?pos is multifield with two float cordinates like this: (?x ?y)
;  ; the node of the final result
;  (navgraph-node (name ?n1) (pos $?pos1))
;  ; make sure there is no other node ?n2 (at ?pos2)  closer to ?pos than ?n1 with ?pos1
;  (not (navgraph-node (name ?n2&~?n1) (pos $?pos2&:(navgraph-closer ?pos ?pos1 ?pos2))))

; 2. Find the node closest to and different from node ?n (with extended comment)
;  ; the node to be closest to, the "M1" constraint is arbitrary and for the sake of
;  ; this example, choose the node by any criterion relevant to your application
;  (navgraph-node (name ?n1&"M1") (pos $?pos1))
;  ; the node of the final result
;  (navgraph-node (name ?n2&~?n1) (pos $?pos2))
;  ; make sure there is no other node (at ?pos3)  closer to ?pos1 than ?n2 with ?pos2
;  (not (navgraph-node (name ~?n1&~?n2) (pos $?pos3&:(navgraph-closer ?pos1 ?pos3 ?pos2))))

; 3. Check for the closest node ?n1 to another node M1 with property "orientation"
;  (navgraph-node (name ?n&"M1") (pos $?pos))
;  (navgraph-node (name ?n1&~?n) (pos $?pos1)
;		 (properties $?props&:(navgraph-has-property ?props "orientation")))
;  (not (navgraph-node (name ?n2&~?n1&~?n) (pos $?pos2&:(navgraph-closer ?pos ?pos1 ?pos2))
;		      (properties $?props2&:(navgraph-has-property ?props2 "orientation"))))

(deftemplate navgraph
  (slot name (type STRING))
)

(deftemplate navgraph-node
  (slot name (type STRING))
  (multislot pos (type FLOAT) (cardinality 2 2))
  (multislot properties (type STRING))
)

(deftemplate navgraph-edge
  (slot from (type STRING))
  (slot to (type STRING))
  (slot directed (type SYMBOL) (allowed-values FALSE TRUE))
  (multislot properties (type STRING))
)

(deffunction navgraph-pos-x (?pos)
  (return (nth$ 1 ?pos))
)

(deffunction navgraph-pos-y (?pos)
  (return (nth$ 2 ?pos))
)

(deffunction navgraph-pos-distance (?pos1 ?pos2)
  (return (sqrt (+ (** (- (navgraph-pos-x ?pos1) (navgraph-pos-x ?pos2)) 2)
		   (** (- (navgraph-pos-y ?pos1) (navgraph-pos-y ?pos2)) 2))))
)

;; Check if a node is closer than another to a common node.
; For three nodes A, B, and C, where we are interested in the distances
; of B to A and C to A, check if B-A is closer than C-A.
; @param ?pos-A position of common node A
; @param ?pos-B position of node B
; @param ?pos-C position of node C
; @return true if the distance B-A is smaller than C-A.
(deffunction navgraph-closer (?pos-A ?pos-B ?pos-C)
  (return (< (navgraph-pos-distance ?pos-A ?pos-B) (navgraph-pos-distance ?pos-A ?pos-C)))
)

;; Find the node closest to the given node position.
; @param ?pos position for which to find the closest node
; @return the name of the closest node or FALSE if non found
(deffunction navgraph-closest-to (?pos)
  (do-for-fact ((?n1 navgraph-node))
	       (and (neq ?n1:pos ?pos)
		    (not (any-factp ((?n2 navgraph-node))
				    (and (neq ?n2:pos ?pos)
					 (neq ?n2:pos ?n1:pos)
					 (navgraph-closer ?pos ?n2:pos ?n1:pos)))))
    (return ?n1:name)
  )
  (return FALSE)
)

;; Find the node closest to a given node.
; @param ?name name of the node to find the closest (but different) node to
; @return the name of the node closest to the given node or FALSE if not found
(deffunction navgraph-closest-by-name (?name)
  (bind ?nodes (find-fact ((?n navgraph-node)) (eq ?n:name ?name)))
  (if (> (length$ ?nodes) 0) then
    (return (navgraph-closest-to (fact-slot-value (nth$ 1 ?nodes) pos)))
  )
  (return FALSE)
)

(deffunction navgraph-has-property (?props ?prop)
  (foreach ?p ?props
    (if (= (mod ?p-index 2) 1) then
      (if (eq ?p ?prop) then (return TRUE))
    )
  )
  (return FALSE)
)

(deffunction navgraph-property (?props ?prop)
  (foreach ?p ?props
    (if (= (mod ?p-index 2) 1) then
      (if (eq ?p ?prop) then (return (nth$ (+ ?p-index 1) ?props)))
    )
  )
  (return FALSE)
)

(deffunction navgraph-property-as-float (?props ?prop)
  (foreach ?p ?props
    (if (= (mod ?p-index 2) 1) then
      (if (eq ?p ?prop) then (return (float (string-to-field (nth$ (+ ?p-index 1) ?props)))))
    )
  )
  (return 0.0)
)

(deffunction navgraph-property-as-int (?props ?prop)
  (foreach ?p ?props
    (if (= (mod ?p-index 2) 1) then
      (if (eq ?p ?prop) then (return (integer (string-to-field (nth$ (+ ?p-index 1) ?props)))))
    )
  )
  (return 0)
)

(deffunction navgraph-property-as-bool (?props ?prop)
  (foreach ?p ?props
    (if (= (mod ?p-index 2) 1) then
      (if (eq ?p ?prop) then
	(if (eq (nth$ (+ ?p-index 1) ?props) "true") then (return TRUE) else (return FALSE))
      )
    )
  )
  (return FALSE)
)

(deffunction navgraph-cleanup ()
  (delayed-do-for-all-facts ((?nn navgraph-node)) TRUE
    (retract ?nn)
  )
  (delayed-do-for-all-facts ((?ne navgraph-edge)) TRUE
    (retract ?ne)
  )
  (delayed-do-for-all-facts ((?ng navgraph)) TRUE
    (retract ?ng)
  )
)
