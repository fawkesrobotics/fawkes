
;---------------------------------------------------------------------------
;  aggregate.clp - CEDAR system monitoring system condition aggregation
;
;  Created: Sat Oct 29 23:49:52 2011
;  Copyright  2011  Tim Niemueller [www.niemueller.de]
;             2011  SRI International
;             2011  RWTH Aachen University (KBSG)
;             2011  Carnegie Mellon University
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defglobal
  ?*CEDAR-SYSCOND-PERIOD* = 1
)

(deffacts cedar-syscond-deffacts
  (timer (name cedar-syscond))
)


(deftemplate system-condition
  (slot id (type SYMBOL))
  (slot cond (type SYMBOL) (allowed-values UNKNOWN GREEN YELLOW RED))
  (slot desc (type STRING))
  (multislot updated (type INTEGER) (cardinality 2 2) (default 0 0))
)

(deffunction cond> (?a ?b)
  (or (and (eq ?a RED)    (neq ?b RED))
      (and (eq ?a YELLOW) (neq ?b RED YELLOW))
      (and (eq ?a GREEN)  (neq ?b RED YELLOW GREEN)))
)

(deffunction syscond> (?a ?b)
  (return (cond> (fact-slot-value ?a cond) (fact-slot-value ?b cond)))
)

(deffunction cond-lt (?a ?b)
  (or (and (eq ?a YELLOW)  (neq ?b YELLOW GREEN UNKNOWN))
      (and (eq ?a GREEN)   (neq ?b GREEN UNKNOWN))
      (and (eq ?a UNKNOWN) (neq ?b UNKNOWN)))
)

(deffunction syscond-lt (?a ?b)
  (return (cond-lt (fact-slot-value ?a cond) (fact-slot-value ?b cond)))
)

(deffunction syscond-join-desc (?descs)
  (bind ?rv "")
  (foreach ?d ?descs
    (if (> ?d-index 1) then (bind ?rv (str-cat ?rv "|")))
    (bind ?rv (str-cat ?rv ?d))
  )
  (return ?rv)
)

; --- RULES - system state
(defrule determine-system-condition
  (time $?now)
  ?tf <- (timer (name cedar-syscond) (time $?t&:(timeout ?now ?t ?*CEDAR-SYSCOND-PERIOD*)))
  =>
  (modify ?tf (time ?now))
  (bind ?conds (create$))
  (do-for-all-facts ((?sc system-condition)) TRUE
                    (bind ?conds (create$ ?conds ?sc)))
  (bind ?conds (sort syscond-lt ?conds))

  ; printout individual sysconds if there are any and debug is enabled
  (if (and (debug 2) (> (length$ ?conds) 0)) then
    (printout t "Individual SysConds: " crlf)
    (foreach ?c ?conds ;(printout t "  " ?c crlf))
      (printout t "  "(fact-slot-value ?c id) ": " (fact-slot-value ?c cond) crlf)
      (if (> (str-length (fact-slot-value ?c desc)) 0) then
	(printout t "  (" (fact-slot-value ?c desc) ")" crlf))
    )
  )

  ; Final condition default (if none reported)
  (bind ?final-cond UNKNOWN)
  (bind ?final-desc "No conditions reported")

  ; Merge final condition
  (if (> (length$ ?conds) 0) then
    (bind ?cond-red (create$))
    (bind ?cond-yellow (create$))
    (bind ?cond-green (create$))
    (bind ?cond-unknown (create$))

    (foreach ?c ?conds ;(printout t "  " ?c crlf))
      (if (eq (fact-slot-value ?c cond) RED) then
	(bind ?final-cond RED)
	(bind ?cond-red (append$ ?cond-red (fact-slot-value ?c id)))
      )
      (if (eq (fact-slot-value ?c cond) YELLOW) then
	(bind ?final-cond YELLOW)
	(bind ?cond-yellow (append$ ?cond-yellow (fact-slot-value ?c id)))
      )
      (if (eq (fact-slot-value ?c cond) GREEN) then
	(bind ?final-cond GREEN)
	(bind ?cond-green (append$ ?cond-green (fact-slot-value ?c id)))
      )
      (if (eq (fact-slot-value ?c cond) UNKNOWN) then
	(bind ?final-cond UNKNOWN)
	(bind ?cond-unknown (append$ ?cond-unknown (fact-slot-value ?c id)))
      )
    )
    (bind ?desc-red "NONE")
    (bind ?desc-yellow "NONE")
    (bind ?desc-green "NONE")
    (bind ?desc-unknown "NONE")
    (if (> (length$ ?cond-red) 0)
     then
      (bind ?final-cond RED)
      (bind ?desc-red (str-cat (length$ ?cond-red) " (" (implode$ ?cond-red) ")"))
     else
      (if (> (length$ ?cond-yellow) 0)
       then
        (bind ?final-cond YELLOW)
	(bind ?desc-yellow (str-cat (length$ ?cond-yellow) " (" (implode$ ?cond-yellow) ")"))
       else
        (if (> (length$ ?cond-green) 0)
         then
          (bind ?final-cond GREEN)
	  (bind ?desc-green (str-cat (length$ ?cond-green) " (" (implode$ ?cond-green) ")"))
         else
          (bind ?final-cond UNKNOWN)
	  (if (> (length$ ?cond-unknown) 0) then
	    (bind ?desc-green (str-cat (length$ ?cond-unknown) " ("
				       (implode$ ?cond-unknown) ")"))
          )
        )
      )
    )
    (bind ?final-desc (str-cat "RED: " ?desc-red "  YELLOW: " ?desc-yellow "  "
			       "GREEN: " ?desc-green "  UNKNOWN: " ?desc-unknown))
  )
  ;(assert (system-condition (id "combined")
  ;                          (cond ?final-cond)
  ;                          (desc ?final-desc)))

  (if (debug 2) then
    (printout t "Combined SysCond:    " ?final-cond crlf "  " ?final-desc crlf)
  )

)


; (defrule notify-system-condition
;   (declare (salience (- ?*PRIORITY_SYSCOND* 1)))
;   ?scf <- (system-condition (id "combined") (cond ?cond) (desc ?desc))
;   =>
;   (if (debug 1) then
;     (printout t "*** System condition: " ?cond " (" ?desc ")" crlf)
;   )
;   (retract ?scf)
;   (notify-system-condition (str-cat ?cond) (now) ?desc)
; )

; (defrule system-condition-cleanup
;   (declare (salience ?*PRIORITY_LAST*))
;   ?scf <- (system-condition)
;   =>
;   (retract ?scf)
; )
