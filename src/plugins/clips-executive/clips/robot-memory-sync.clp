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

(defrule robot-memory-sync-add
  "Add new facts to robot memory."
  ?f <- (domain-fact)
  =>
  (robmem-insert "robmem.clipswm" (rm-structured-fact-to-bson ?f))
)

(defrule robot-memory-sync-retract
  "Remove deleted facts from robot memory."
  (declare (salience 100))
  ?f <- (domain-fact (name ?name) (param-values $?param-values))
  ?p <- (domain-retracted-fact (name ?name) (param-values $?param-values))
  =>
  (robmem-remove "robmem.clipswm" (rm-structured-fact-to-bson ?f))
)
