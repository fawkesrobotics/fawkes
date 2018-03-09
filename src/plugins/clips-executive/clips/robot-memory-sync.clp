;---------------------------------------------------------------------------
;  robot-memory-sync.clp - Synchronize domain model with robot memory
;
;  Created: Thu 30 Nov 2017 19:30:50 CET
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

; This is a simple way to synchronize the domain model with robot memory. All
; new facts in the domain model are inserted into robot memory, all retracted
; facts are removed from robot memory.  We may or may not use this in the
; future, but it serves its purpose for now.

(deftemplate robot-memory-sync-mapped
	(slot name (type SYMBOL))
	(multislot param-values)
)

(deffunction robot-memory-sync-clean-domain-facts
  "Remove all domain facts from the database so it is consistent with CLIPS."
  ()
  (printout warn "Clearing domain facts from robot memory" crlf)
  (bind ?doc (bson-create))
  (bson-append ?doc "relation" domain-fact)
  (robmem-remove "robmem.clipswm" ?doc)
)

(defrule robot-memory-sync-add-object
  "Add new facts to robot memory."
  (declare (salience 100))
  ?f <- (domain-object)
  =>
  (bind ?bson (rm-structured-fact-to-bson ?f))
  (robmem-upsert "robmem.clipswm" ?bson ?bson)
)

(defrule robot-memory-sync-add-fact
  "Add new facts to robot memory."
  (declare (salience 100))
  ?f <- (domain-fact (name ?name) (param-values $?param-values))
	(not (robot-memory-sync-mapped (name ?name) (param-values $?param-values)))
  =>
	(assert (robot-memory-sync-mapped (name ?name) (param-values ?param-values)))
  (bind ?bson (rm-structured-fact-to-bson ?f))
  (robmem-upsert "robmem.clipswm" ?bson ?bson)
)

(defrule robot-memory-sync-retract-fact
  "Remove deleted facts from robot memory."
  (declare (salience 100))
	?mf <- (robot-memory-sync-mapped (name ?name) (param-values $?param-values))
  (not (domain-fact (name ?name) (param-values $?param-values)))
  =>
  (robmem-remove "robmem.clipswm" (rm-structured-fact-to-bson ?mf "domain-fact"))
	(retract ?mf)
)
