
;---------------------------------------------------------------------------
;  utils.clp - CLIPS agent utility functions
;
;  Created: Sun Jun 17 12:19:34 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*DEBUG* = 2  ;debug levels: 0 ~ none, 1 ~ minimal, 2 ~ more, 3 ~ maximum
)

(deffunction debug-set-level (?level)
  (bind ?*DEBUG* ?level)
)

(deffunction debug (?level)
  (return (<= ?level ?*DEBUG*))
)

(deffunction append$ (?list $?items)
  (insert$ ?list (+ (length$ ?list) 1) ?items)
)

(deffunction randomize$ (?list)
  (bind ?l (length$ ?list))
  (loop-for-count 200 do
    (bind ?a (random 1 ?l))
    (bind ?b (random 1 ?l))
    (bind ?tmp (nth$ ?a ?list))
    (bind ?list (replace$ ?list ?a ?a (nth$ ?b ?list)))
    (bind ?list (replace$ ?list ?b ?b ?tmp))
  )
  (return ?list)
)

; Set equality, i.e. a in b and b in a.
(deffunction set-eq (?a ?b)
  (return (and (subsetp ?a ?b) (subsetp ?b ?a)))
)

; Get set difference ?a \ ?b, i.e. elements which exist in ?a but not in ?b
(deffunction set-diff (?a ?b)
  (bind ?rv (create$))
  (foreach ?e ?a (if (not (member$ ?e ?b)) then (bind ?rv (append$ ?rv ?e))))
  (return ?rv)
)

(deffunction is-even-int (?num)
  (return (eq (mod ?num 2) 0))
)

(deffunction is-odd-int (?num)
  (return (eq (mod ?num 2) 1))
)

(deffunction str-replace (?s ?search ?replace)
  (bind ?i (str-index ?search ?s))
  (bind ?l (str-length ?search))
  (if (eq ?i FALSE)
   then (return ?s)
   else
    (return (str-cat (sub-string 1 (- ?i 1) ?s) ?replace (sub-string (+ ?i ?l) (str-length ?s) ?s)))
  )
)

;; String "greater than" comparison.
; @param ?a string a to compare
; @param ?b string b to compare
; @return true if (str-compare ?a ?b) is greater than thero, false otherwise
(deffunction string> (?a ?b) (> (str-compare ?a ?b) 0))

(deffunction str-prefix (?prefix ?s)
	"Check if ?s starts with ?prefix
   @param ?s string to check
   @param ?prefix string to check for in ?s
   @return TRUE if ?s starts with ?prefix, FALSE otherwise"
	(bind ?p (str-index ?prefix ?s))
	(if ?p then (return (= ?p 1)))
	(return FALSE)
)
