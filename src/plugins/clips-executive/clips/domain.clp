;---------------------------------------------------------------------------
;  domain.clp - Representation of a planning domain
;
;  Created: Fri 22 Sep 2017 11:35:49 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate obj-type
  "A type in the domain. The type obj must be super-type of all types."
  (slot name (type SYMBOL))
  (slot super-type (type SYMBOL) (default obj))
)

(deftemplate dom-object
  "An object in the domain with the given name and type. The type must refer to
   the name of an existing type."
  (slot name)
  (slot obj-type (type SYMBOL) (default obj))
)

(deftemplate predicate
  "A predicate symbol in the domain. If a predicate exists, it is true,
   otherwise it is false."
  (slot name (type SYMBOL) (default ?NONE))
  (multislot parameters (default (create$)))
)

(deftemplate operator
  "An operator of the domain. This only defines the name of the operator, all
   other properties (parameters, precondition, effects) are defined in separate
   templates."
  (slot name (type SYMBOL))
)

(deftemplate parameter
  "A parameter of an operator. The operator and obj-type slots must refer to the
   names of an existing operator and an existing type respectively."
  (slot name)
  (slot operator (type SYMBOL))
  (slot obj-type (type SYMBOL) (default obj))
)

(deftemplate precondition
  "A (non-atomic) precondition of an operator. Must be either part-of an
   operator or another precondition. Use the name to assign other preconditions
   as part of this precondition. This can currently be a conjunction or a
   negation. If it is a negation, it can have only one sub-condition. If it is
   a conjunction, it can have an arbitrary number of sub-conditions."
  (slot part-of (type SYMBOL))
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot type (type SYMBOL) (allowed-values conjunction negation))
)

(deftemplate atomic-precondition
  "An atomic precondition of an operator. This must always be part-of a
   non-atomic precondition."
  (slot part-of (type SYMBOL))
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot predicate (type SYMBOL))
  (multislot parameters (type SYMBOL))
)

(deftemplate action
  (slot operator (type SYMBOL))
)

(deftemplate grounding
  "A grounding of a single parameter of an operator"
  (slot operator (type SYMBOL))
  (slot parameter (type SYMBOL))
  (slot value)
)
