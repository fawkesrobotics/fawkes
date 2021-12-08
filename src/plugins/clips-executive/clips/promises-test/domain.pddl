;****************************************************************************
;  domain.pddl: Test domain for the promises
;
;  Created: Thu Dec 2 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;****************************************************************************

;  This program is free software; you can redistribute it and/or modify
;  it under the terms of the GNU General Public License as published by
;  the Free Software Foundation; either version 2 of the License, or
;  (at your option) any later version.
;
;  This program is distributed in the hope that it will be useful,
;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;  GNU Library General Public License for more details.
;
;  Read the full text in the LICENSE.GPL file in the doc directory.

(define (domain promises-test)
	(:requirements :strips :typing)

	(:types
		robot - object
		location - object
		container - object
		machine - object
		machine-state - object
		material - object
	)

	(:constants
		BASE MACHINE1-INPUT MACHINE1-OUTPUT MACHINE2-INPUT MACHINE2-OUTPUT REGOLITH-MINE1 REGOLITH-MINE2 STORAGE-INPUT CONTAINER-DEPOT - location
		MACHINE1 MACHINE2 - machine
		C1 C2 C3 C4 - container
		IDLE FILLED OPERATING READY - machine-state
		REGOLITH XENONITE PROCESSITE BYPRODUCT - material
	)

	(:predicates
		(self ?r - robot)
		(robot-at ?r - robot ?l - location)
		(robot-carries ?r - robot ?c - container)
		(robot-can-carry ?r - robot)
		(container-at ?c - container ?l - location)
		(container-filled ?c - container ?m - material)
		(container-can-be-filled ?c - container)
		(container-for-robot ?c -container ?r -robot)
		(machine-in-state ?m - machine ?s - machine-state)
		(machine-for-material ?m - machine ?mat - material)
		(machine-makes-material ?m - machine ?mat - material)
		(location-is-mine ?location - location)
		(location-part-of-machine ?location - location ?machine - machine)
		(location-is-machine-input ?location - location)
		(location-is-machine-output ?location - location)
		(location-is-free ?location - location)
		(storage-is-full)
	)

	(:action collect-regolith
		:parameters (?r - robot ?mine - location ?c - container)
		:precondition (and (location-is-mine ?mine) (robot-at ?r ?mine) (robot-carries ?r ?c) (container-can-be-filled ?c))
		:effect (and (not (container-can-be-filled ?c)) (container-filled ?c REGOLITH))
	)

	(:action put-regolith
		:parameters (?r - robot ?side - location ?machine - machine ?c - container)
		:precondition (and (location-part-of-machine ?side ?machine) (location-is-machine-input ?side) (machine-in-state ?machine IDLE) (robot-at ?r ?side) (robot-carries ?r ?c) (container-filled ?c REGOLITH) (machine-for-material ?machine REGOLITH))
		:effect (and (not (container-filled ?c REGOLITH)) (container-can-be-filled ?c) (machine-in-state ?machine FILLED) (not (machine-in-state ?machine IDLE)))
	)

	(:action pick-container
		:parameters (?r - robot ?c - container)
		:precondition (and (robot-at ?r CONTAINER-DEPOT) (robot-can-carry ?r) (container-at ?c CONTAINER-DEPOT))
		:effect (and (not (robot-can-carry ?r)) (robot-carries ?r ?c) (not (container-at ?c CONTAINER-DEPOT)))
	)

	(:action put-container
		:parameters (?r - robot ?c - container ?side - location)
		:precondition (and (robot-at ?r ?side) (robot-carries ?r ?c))
		:effect (and (not (robot-carries ?r ?c)) (container-at ?c ?side))
	)

	(:action move
		:parameters (?r - robot ?l1 - location ?l2 - location)
		:precondition (and (robot-at ?r ?l1) (location-is-free ?l2))
		:effect (and (not (robot-at ?r ?l1)) (not (location-is-free ?l2)) (robot-at ?r ?l2) (location-is-free ?l1))
	)

	(:action move-plaza
		:parameters (?r - robot ?l1 - location ?l2 - location)
		:precondition (and (robot-at ?r ?l1))
		:effect (and (not (robot-at ?r ?l1)) (robot-at ?r ?l2) (location-is-free ?l1))
	)

	(:action move-plaza-plaza
		:parameters (?r - robot ?l1 - location ?l2 - location)
		:precondition (and (robot-at ?r ?l1))
		:effect (and (not (robot-at ?r ?l1)) (robot-at ?r ?l2))
	)

	(:action start-machine
		:parameters (?m - machine)
		:precondition (and (machine-in-state ?m FILLED))
		:effect (and (machine-in-state ?m OPERATING) (not (machine-in-state ?m FILLED)))
	)

	(:action collect-processite
		:parameters (?r - robot ?machine - machine ?output - location ?c - container)
		:precondition (and (location-is-machine-output ?output) (machine-in-state ?machine READY) (robot-at ?r ?output) (robot-carries ?r ?c) (container-can-be-filled ?c) (machine-makes-material ?machine PROCESSITE))
		:effect (and (not (container-can-be-filled ?c)) (container-filled ?c PROCESSITE) (not (machine-in-state ?machine READY)) (machine-in-state ?machine IDLE))
	)

	(:action put-processite
		:parameters (?r - robot ?side - location ?machine - machine ?c - container)
		:precondition (and (location-part-of-machine ?side ?machine) (location-is-machine-input ?side) (machine-in-state ?machine IDLE) (robot-at ?r ?side) (robot-carries ?r ?c) (container-filled ?c PROCESSITE) (machine-for-material ?machine PROCESSITE))
		:effect (and (not (container-filled ?c PROCESSITE)) (container-can-be-filled ?c) (machine-in-state ?machine FILLED) (not (machine-in-state ?machine IDLE)))
	)

	(:action collect-xenonite
		:parameters (?r - robot ?machine - machine ?output - location ?c - container)
		:precondition (and (location-is-machine-output ?output) (machine-in-state ?machine READY) (robot-at ?r ?output) (robot-carries ?r ?c) (container-can-be-filled ?c) (machine-makes-material ?machine XENONITE))
		:effect (and (not (container-can-be-filled ?c)) (container-filled ?c XENONITE) (not (machine-in-state ?machine READY)) (machine-in-state ?machine IDLE))
	)
)
