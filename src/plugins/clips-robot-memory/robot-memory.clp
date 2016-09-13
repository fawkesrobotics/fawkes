
;---------------------------------------------------------------------------
;  robot-memory.clp - CLIPS robot memory helper functions
;
;  Created: Tue Sep 13 17:24:53 2016
;  Copyright  2016  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;; Creates a BSON document from a structured fact
; @param ?fact Fact-Pointer
; @return BSON document
(deffunction rm-structured-fact-to-bson (?fact)
  (bind ?doc (bson-create))
  (bind ?templ (fact-relation ?fact))
  (bson-append ?doc "relation" (str-cat ?templ))
  ;append kv-pair for each field
  (progn$ (?slot (fact-slot-names ?fact))
    (if (deftemplate-slot-multip ?templ ?slot)
      then
      ; append multifield as array
      (bson-append-array ?doc ?slot (fact-slot-value ?fact ?slot))
      else
      ; appand value directly for singlefields
      (bson-append ?doc ?slot (fact-slot-value ?fact ?slot))
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
  (bson-append ?doc "relation" (str-cat ?templ))
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
    (bind ?relation (bson-get ?doc "relation"))
    else
    (printout error "Can not create fact from " (bson-tostring ?doc) crlf)
    (return)
  )
  (if (member$ ?relation (get-deftemplate-list *))
    then ;structured fact
    (progn$ (?slot ?keys)
      (if (deftemplate-slot-existp ?relation ?slot) then
        (if (deftemplate-slot-multip ?relation ?slot)
          then
          (bind ?values (str-cat ?values "(" ?relation " " (bson-get ?doc ?slot) ")"))
          else
          (bind ?values (str-cat ?values "(" ?relation " " (implode$ (bson-get-array ?doc ?slot)) ")"))
        )
      )
    )
    else ;ordered fact
    (if (member$ "values" ?keys) then
      (bind ?values (str-cat ?values "(" ?relation " " (implode$ (bson-get-array ?doc "values")) ")"))
    )
  )
  
  (assert-string (str-cat "(" ?relation " " ?values ")"))
)
