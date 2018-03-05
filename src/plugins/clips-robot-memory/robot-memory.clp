
;---------------------------------------------------------------------------
;  robot-memory.clp - CLIPS robot memory helper functions
;
;  Created: Tue Sep 13 17:24:53 2016
;  Copyright  2016  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate robmem-trigger
  (slot name (type STRING))
  (multislot rcvd-at (type INTEGER) (cardinality 2 2))
  (slot ptr (type EXTERNAL-ADDRESS))
)

(deffunction str-replace (?input ?sub ?rep)
  "Replace all occurrences of ?sub in ?input by ?rep."
  (bind ?occ (str-index ?sub ?input ))
  (if (eq ?occ FALSE) then
    (return ?input)
  )
  (bind ?prefix (sub-string 1 (- ?occ 1) ?input))
  (bind ?suffix (sub-string (+ ?occ 1) (str-length ?input) ?input))
  (return (str-cat ?prefix ?rep ?suffix))
)

;; Creates a BSON document from a structured fact
; @param ?fact Fact-Pointer
; @param ?relation optional relation name
; @return BSON document
(deffunction rm-structured-fact-to-bson (?fact $?relation)
  (bind ?doc (bson-create))
  (bind ?templ (fact-relation ?fact))
	(if (> (length$ ?relation) 0)
	 then (bson-append ?doc "relation" (sym-cat (nth$ 1 ?relation)))
	 else (bson-append ?doc "relation" (sym-cat ?templ)))
  ;append kv-pair for each field
  (progn$ (?slot (fact-slot-names ?fact))
    (if (deftemplate-slot-multip ?templ ?slot)
      then
      ; append multifield as array
      (bson-append-array ?doc (str-replace ?slot "-" "_") (fact-slot-value ?fact ?slot))
      else
      ; appand value directly for singlefields
      (bson-append ?doc (str-replace ?slot "-" "_") (fact-slot-value ?fact ?slot))
    )
  )      
  (return ?doc)
)

;; Creates a BSON document from an ordered fact
; @param ?fact Fact-Pointer
; @return BSON document
(deffunction rm-ordered-fact-to-bson (?fact)
  (bind ?doc (bson-create))
  (bind ?templ (fact-relation ?fact))
  (bson-append ?doc "relation" (sym-cat ?templ))
  ;append values as array
  (bson-append-array ?doc "values" (fact-slot-value ?fact implied))
  (return ?doc)
)

;; Assert (structured/ordered) fact from a BSON document
; @param ?doc BSON document
(deffunction rm-assert-from-bson (?doc)
  (bind ?relation "")
  (bind ?values "")
  (bind ?keys (bson-field-names ?doc))
  (if (member$ "relation" ?keys)
    then
    (bind ?relation (sym-cat (bson-get ?doc "relation")))
    else
    (printout error "Can not create fact from " (bson-tostring ?doc) crlf)
    (return)
  )
  (if (member$ ?relation (get-deftemplate-list MAIN))
    then ;structured fact
    (progn$ (?slot ?keys)
      (if (deftemplate-slot-existp ?relation (sym-cat ?slot)) then
        (if (deftemplate-slot-multip ?relation (sym-cat ?slot))
	  then
	  (bind ?arr-str (bson-get-array ?doc ?slot))
	  (bind ?arr (create$))
	  (progn$ (?i ?arr-str)
	    (bind ?arr (create$ ?arr (sym-cat ?i)))
	  )
          (bind ?values (str-cat ?values "(" ?slot " " (implode$ ?arr) ")"))
          else
          (bind ?values (str-cat ?values "(" ?slot " " (bson-get ?doc ?slot) ")"))
        )
      )
    )
    else ;ordered fact
    (if (member$ "values" ?keys) then
      (bind ?values (str-cat ?values "(" ?relation " " (implode$ (bson-get-array ?doc "values")) ")"))
    )
  )
  ;(printout t "Asserting:" (str-cat "(" ?relation " " ?values ")") crlf)
  (assert-string (str-cat "(" ?relation " " ?values ")"))
)

(defrule rm-cleanup-trigger-facts
  "remove all trigger update facts when they were not used (by a rule with a higher priority in this iteration)"
  (declare (salience -4000))
  ?rt <- (robmem-trigger (ptr ?bson))
  =>
  (retract ?rt)
  (bson-destroy ?bson)
)
