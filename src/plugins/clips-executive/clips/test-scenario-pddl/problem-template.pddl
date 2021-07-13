;---------------------------------------------------------------------------
;  problem.pddl - A simple Hello World problem
;
;  Created: Thu 30 Nov 2017 10:31:32 CET
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

(define (problem hello-world-problem)
  (:domain hello-world)
  (:objects
    <<#DOMAINOBJECTS|{"relation": "domain-object"}>>
    <<name>> - <<type>><</DOMAINOBJECTS>>
  )
  (:init
    <<#DOMAINFACTS|{"relation": "domain-fact"}>>(<<name>> <<param_values>>)
    <</DOMAINFACTS>>
  )
  (:goal
    <<GOAL>>
  )
)
