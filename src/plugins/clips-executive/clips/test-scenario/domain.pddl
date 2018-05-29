;---------------------------------------------------------------------------
;  domain.pddl - A simple hello world domain
;
;  Created: Wed 29 Nov 2017 15:42:46 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;

(define (domain hello-world)
  (:requirements :strips :typing)
  (:types name text howoften)
  (:predicates
	 (said ?n - name ?t - text)
	 (spoken ?h - howoften)
	 (locked ?n - name)
  )
  (:action say-hello
    :parameters (?name - name)
    :precondition (not (said ?name hello))
    :effect (and (said ?name hello) (spoken once))
  )
  (:action lock
    :parameters (?name - name)
    :precondition (not (locked ?name))
    :effect (and (locked ?name))
  )
  (:action unlock
    :parameters (?name - name)
    :precondition (locked ?name)
    :effect (and (not (locked ?name)))
  )
  (:action say-goodbye
    :parameters (?name - name)
    :precondition (said ?name hello)
    :effect (said ?name goodbye)
	)
  (:action say-hello-again
    :parameters (?name - name)
    :precondition (not (said ?name hello))
    :effect (and (said ?name hello) (not (spoken once)))
  )
	;(:action print
	;	:parameters (?severity - text ?text - text)
	;)
	;(:action say-cleanup)
)
