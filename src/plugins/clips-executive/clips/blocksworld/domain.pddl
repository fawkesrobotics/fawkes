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

(define (domain blocksworld)
  (:requirements :strips :typing)
  (:types name text howoften block robot)
  
  (:constants
    hello goodbye - text
    once - howoften
  )
  (:predicates
	 (said ?n - name ?t - text)
	 (spoken ?h - howoften)
	 (locked ?n - name)
   (on ?x - block ?y - block)
   (ontable ?x - block)
   (clear ?x - block)
   (handempty ?x - robot)
   (handfull ?x - robot)
   (holding ?x - block)
   (pickup ?x - block)
   (putdown ?x - block)
   (stack ?x - block ?y - block)
   (unstack ?x - block)
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
    :precondition (said ?name hello)
    :effect (and (said ?name hello) (not (spoken once)))
  )
	(:action pick-up
        :parameters (?x - block ?r - robot)
        :precondition (and
            (pickup ?x) 
            (clear ?x) 
            (ontable ?x) 
            (handempty ?r)
        )
        :effect (and
            (not (ontable ?x))
            (not (clear ?x))
            (not (handempty ?r))
            (handfull ?r)
            (holding ?x)
        )
    )

    (:action put-down
        :parameters (?x - block ?robot - robot)
        :precondition (and 
            (putdown ?x)
            (holding ?x)
            (handfull ?robot)
        )
        :effect (and 
            (not (holding ?x))
            (clear ?x)
            (handempty ?robot)
            (not (handfull ?robot))
            (ontable ?x))
        )

    (:action stack
        :parameters (?x - block ?y - block ?robot - robot)
        :precondition (and
            (stack ?x ?y)
            (holding ?x) 
            (clear ?y)
            (handfull ?robot)
        )
        :effect (and 
            (not (holding ?x))
            (not (clear ?y))
            (clear ?x)
            (handempty ?robot)
            (not (handfull ?robot))
            (on ?x ?y)
        )
    )

    (:action unstack
        :parameters (?x - block ?y - block ?robot - robot)
        :precondition (and
            (unstack ?x)
            (on ?x ?y)
            (clear ?x)
            (handempty ?robot)
        )
        :effect (and 
            (holding ?x)
            (clear ?y)
            (not (clear ?x))
            (not (handempty ?robot))
            (handfull ?robot)
            (not (on ?x ?y))
        )
    )
)
