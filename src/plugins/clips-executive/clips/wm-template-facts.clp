;---------------------------------------------------------------------------
;  wm-template-facts.clp - helpers to sync template facts to the world model
;
;  Copyright  2022  Tarik Viehmann
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; World model facts are a key component of the CLIPS executive and offer
; a unified storage structure, which enables features such as world model
; synchronization.
;
; However, agents may also rely on other template facts tailored to specific
; use cases and with a more convenient structure.
; This file offers utility functions to translate template facts to
; world model facts and vice versa.
;
; Example:
; Consider a distributed system, where goals should be shared across agents.
; The following rule can translate a goal to a world model fact:
;
; (defrule goal-to-wm-fact
;   ?g <- (goal (id ?id))
;   (not (wm-fact (key template fact goal id ?id $?)))
; =>
;   (assert (wm-fact (key (template-fact-to-wm-key ?g
;                                                  id
;                                                  (deftemplate-remaining-slots goal id)))))
; )

(deffunction deftemplate-remaining-slots (?template ?slots)
" For a given template, return all slots not specified in ?slots."
	(if (neq (type ?slots) MULTIFIELD)
	 then
		(bind ?slots (create$ ?slots))
	)
	(bind ?res (deftemplate-slot-names goal))
	(progn$ (?slot ?slots)
		(bind ?pos (member$ ?slot ?res))
		(bind ?res (delete$ ?res ?pos ?pos))
	)
	(return ?res)
)

(deffunction value-to-type-pair (?value)
	(return (create$ (type ?value) (sym-cat ?value)))
)

(deffunction values-to-type-pairs (?list)
	(bind ?res (create$))
	(foreach ?c ?list
		(bind ?res (append$ ?res (value-to-type-pair ?c)))
	)
	(return ?res)
)

(deffunction type-cast (?value ?type)
" Convert a value to a given type by a direct type cast.
  @param ?value Value to cast
  @param ?type Target type to cast
  @return Value of ?value with ?type
"
	; If the current type is a symbol we can simply use string-to-field to cast.
	; However, if the target type is a STRING, whitespaces may become a problem.
	(if (and (neq ?type STRING) (eq (type ?value) SYMBOL))  then
		(bind ?value (string-to-field ?value))
		(return ?value)
	)

	(switch ?type
		(case INTEGER then (return (integer ?value)))
		(case STRING then (return (str-cat ?value)))
		(case FLOAT then (return (float ?value)))
		(case SYMBOL then (return (sym-cat ?value)))
	)
	(printout t "cast: unsupported type cast: " ?type " Expected INTEGER|STRING|FLOAT|SYMBOL" crlf)
	(return ?value)
)

(deffunction slots-to-multifield (?fact-id ?slots)
" Concert a list of slots of a template to a multifield of SYMBOLS that can
  be part of a wm-fact key
"
	(bind ?template (fact-relation ?fact-id))
	(bind ?res (create$))
	(loop-for-count (?i 1 (length$ ?slots))
		(bind ?curr-slot (nth$ ?i ?slots))
		(bind ?res (append$ ?res ?curr-slot))
		(if (deftemplate-slot-multip ?template ?curr-slot)
		 then
			(bind ?res (append$ ?res (create$ [ (values-to-type-pairs (fact-slot-value ?fact-id ?curr-slot)) ])))
		 else
			(bind ?types (deftemplate-slot-types ?template ?curr-slot))
			(if (neq (length$ ?types) 1)
			 then
				(printout t "slots-to-multifield: type of slot " ?curr-slot
				                " of template "  ?template " cannot be determined, skipping." crlf)
					(bind ?res (delete$ ?res (length$ ?res) (length$ ?res)))
				else
					(bind ?res (append$ ?res (fact-slot-value ?fact-id ?curr-slot)))
			)
		)
	)
	(return ?res)
)

(deffunction template-fact-to-wm-key (?fact-id ?id-slots ?arg-slots)
" Encode a fact to a wm-fact key.

  Slots are encoded by <slot-name> <slot-value> and slots are required to have
  a single allowed type.
  Multislots are encoded by <slot-name> [ <slot-value-type> <slot-value> ... ]
  to ensure type safety when converting back via template-fact-str-from-wm-key.

  @param ?fact-id:   id of fact to encode as wm-fact key
  @param ?id-slots:  slot names that uniquely identify facts of the template
                     Those will be part of the base ID.
  @param ?arg-slots: other slot names that should be captured by the wm-fact key
"
	(bind ?template (fact-relation ?fact-id))
	(bind ?key (create$ template fact ?template))
	(if (neq (type ?id-slots) MULTIFIELD) then (bind ?id-slots (create$ ?id-slots)))
	(bind ?key (append$ ?key (slots-to-multifield ?fact-id ?id-slots)))
	(if (neq (type ?arg-slots) MULTIFIELD) then (bind ?arg-slots (create$ ?arg-slots)))

	(if (> (length$ ?arg-slots) 0) then
		(bind ?key (append$ ?key args?))
		(bind ?key (append$ ?key (slots-to-multifield ?fact-id ?arg-slots)))
	)
	(return ?key)
)

(deffunction template-fact-str-from-wm-key (?key)
" Build a fact string from a template fact wm-fact key.
"
	(if (eq (subseq$ ?key 1 2) (create$ template fact))
	 then
		(bind ?template (nth$ 3 ?key))
		(bind ?key (delete$ ?key 1 3))
		; when converting back there is no distinction between id-slots and
		; arg-slots, hence remove the args? separator if present
		(bind ?args-pos (member$ args? ?key))
		(if ?args-pos then
			(bind ?key (delete$ ?key ?args-pos ?args-pos))
		)
		; start with the template name ...
		(bind ?res (str-cat "(" ?template))
		(bind ?mode SLOT)
		(bind ?slot nil)
		(bind ?type SYMBOL)
		(bind ?is-list FALSE)
		; remember to terminate strings with \" if necessary
		(bind ?string-active FALSE)
		; and now consider all the encoded slots and multislots
		(progn$ (?e ?key)
			; correctly terminate a multislot, if ] is parsed
			(if (and ?is-list (eq ?e ]))
			 then
				(bind ?mode SLOT)
				(if (eq ?type STRING)
				 then
					(bind ?res (str-cat ?res "\""))
					(bind ?string-active FALSE)
				)
				(bind ?res (str-cat ?res "))"))
			 else
				(switch ?mode
					; if a slot name is expected, add it to the fact string
					(case SLOT then
						 (bind ?is-list FALSE)
						(bind ?res (str-cat ?res " (" ?e ))
						(bind ?slot ?e)
						(if (deftemplate-slot-multip ?template ?slot)
						 then
							; ... and prepare to read a type, if it is a multislot ...
							(bind ?mode TYPE)
						 else
							; ... or prepare to read a type, if it is a normal slot
							(bind ?mode VALUE)
							(bind ?type (nth$ 1 (deftemplate-slot-types ?template ?slot)))
						)
					)
					; if a type is expected, add it and expect a VALUE next
					(case TYPE then
						(bind ?type SYMBOL)
						(if (eq ?e [)
						 then
							(bind ?is-list TRUE)
							(bind ?res (str-cat ?res " (create$ "))
						 else
							(bind ?type ?e)
							(bind ?mode VALUE)
						)
					)
					; if a value is expected, add it ...
					(case VALUE then
						(bind ?res (str-cat ?res " "))
						(if (and (eq ?type STRING) (eq ?string-active FALSE))
						 then
							(bind ?res (str-cat ?res "\""))
							(bind ?string-active TRUE)
						)
						(bind ?res (str-cat ?res (type-cast ?e ?type)))
						; ... and terminate brackets and strings if needed
						(if (> (length$ ?key) ?e-index)
						 then
							(if (member$ (nth$ (+ 1 ?e-index) ?key) (deftemplate-slot-types ?template ?slot))
							 then
								(bind ?mode TYPE)
								(if (eq ?type STRING)
								 then
									(bind ?res (str-cat ?res "\""))
									(bind ?string-active FALSE)
								)
							 else
								(if (member$ (nth$ (+ 1 ?e-index) ?key) (deftemplate-slot-names ?template))
								 then
									(bind ?mode SLOT)
									(if (eq ?type STRING)
									 then
										(bind ?res (str-cat ?res "\""))
										(bind ?string-active FALSE)
									)
									(bind ?res (str-cat ?res ")"))
								)
							)
						 else
							(if (eq ?type STRING)
							 then
								(bind ?res (str-cat ?res "\""))
								(bind ?string-active FALSE)
							)
							(bind ?res (str-cat ?res ")"))
						)
					)
				)
			)
		)
		(bind ?res (str-cat ?res ")"))
		(return ?res)
	 else
		(printout t "template-fact-str-from-wm-key: supplied wm-fact key " ?key
		            " does not encode a template fact, abort." crlf)
		(return "")
	)
)
