;---------------------------------------------------------------------------
;  domain.clp - Domain configuration
;
;  Created: Thu Dec 2 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

(defrule domain-load
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "promises-test/domain.pddl"))
  ;(assert (skiller-control))
  (assert (domain-loaded))
)

(defrule domain-load-initial-facts
" Load all initial domain facts on startup of the game "
  (domain-loaded)
  =>
  (printout info "Initializing domain" crlf)

  (assert
	  (domain-object (name WallE) (type robot))
    (domain-object (name Eve) (type robot))
    (domain-object (name R2D2) (type robot))
    (domain-object (name Arnie) (type robot))
    (domain-fact (name robot-at) (param-values WallE BASE))
    (domain-fact (name robot-at) (param-values Eve BASE))
    (domain-fact (name robot-at) (param-values R2D2 BASE))
    (domain-fact (name robot-at) (param-values Arnie BASE))
    (domain-fact (name robot-can-carry) (param-values WallE))
    (domain-fact (name robot-can-carry) (param-values Eve))
    (domain-fact (name robot-can-carry) (param-values R2D2))
    (domain-fact (name robot-can-carry) (param-values Arnie))

    (domain-fact (name container-can-be-filled) (param-values C1))
    (domain-fact (name container-can-be-filled) (param-values C2))
    (domain-fact (name container-can-be-filled) (param-values C3))
    (domain-fact (name container-can-be-filled) (param-values C4))
    (domain-fact (name container-at) (param-values C1 CONTAINER-DEPOT))
    (domain-fact (name container-at) (param-values C2 CONTAINER-DEPOT))
    (domain-fact (name container-at) (param-values C3 CONTAINER-DEPOT))
    (domain-fact (name container-at) (param-values C4 CONTAINER-DEPOT))
    (domain-fact (name container-for-robot) (param-values C1 WallE))
    (domain-fact (name container-for-robot) (param-values C2 Eve))
    (domain-fact (name container-for-robot) (param-values C3 R2D2))
    (domain-fact (name container-for-robot) (param-values C4 Arnie))

    (domain-fact (name machine-in-state) (param-values MACHINE1 IDLE))
    (domain-fact (name machine-in-state) (param-values MACHINE2 IDLE))
    (domain-fact (name machine-for-material) (param-values MACHINE1 REGOLITH))
    (domain-fact (name machine-for-material) (param-values MACHINE2 PROCESSITE))
    (domain-fact (name machine-makes-material) (param-values MACHINE1 PROCESSITE))
    (domain-fact (name machine-makes-material) (param-values MACHINE2 XENONITE))

    (domain-fact (name location-is-mine) (param-values REGOLITH-MINE1))
    (domain-fact (name location-is-mine) (param-values REGOLITH-MINE2))
    (domain-fact (name location-is-machine) (param-values MACHINE1-INPUT))
    (domain-fact (name location-is-machine) (param-values MACHINE1-OUTPUT))
    (domain-fact (name location-is-machine) (param-values MACHINE2-INPUT))
    (domain-fact (name location-is-machine) (param-values MACHINE2-OUTPUT))
    (domain-fact (name location-is-machine-input) (param-values MACHINE1-INPUT))
    (domain-fact (name location-is-machine-output) (param-values MACHINE1-OUTPUT))
    (domain-fact (name location-is-machine-input) (param-values MACHINE2-INPUT))
    (domain-fact (name location-is-machine-output) (param-values MACHINE2-OUTPUT))
    (domain-fact (name location-is-free) (param-values REGOLITH-MINE1))
    (domain-fact (name location-is-free) (param-values REGOLITH-MINE2))
    (domain-fact (name location-is-free) (param-values MACHINE1-INPUT))
    (domain-fact (name location-is-free) (param-values MACHINE1-OUTPUT))
    (domain-fact (name location-is-free) (param-values MACHINE2-INPUT))
    (domain-fact (name location-is-free) (param-values MACHINE2-OUTPUT))
    (domain-fact (name location-is-free) (param-values STORAGE-INPUT))
    (domain-fact (name location-part-of-machine) (param-values MACHINE1-INPUT MACHINE1))
    (domain-fact (name location-part-of-machine) (param-values MACHINE1-OUTPUT MACHINE1))
    (domain-fact (name location-part-of-machine) (param-values MACHINE2-INPUT MACHINE2))
    (domain-fact (name location-part-of-machine) (param-values MACHINE2-OUTPUT MACHINE2))
  )

  (assert (domain-facts-loaded))
)

