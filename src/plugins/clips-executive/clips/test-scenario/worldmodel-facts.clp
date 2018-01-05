(defrule test-wm-facts
	(executive-init)
	=>
	(assert
		; the following could be a natural representation in the world model.
		; However, in a domain fact context, where the last element corresponds
		; to the predicate name, this can lead to inconsistent, i.e., non-unique
		; elements in the world model.
		;(wm-fact (key "/perception/person-in-front") (value "Peter"))

		; The following is for a two-ary predicate location. However, this
		; here only works because the location of a robot is unique, but
		; consider a "loaded" predicate indicating what a transport robot
	  ; is transporting. This may no longer be unique.
		;(wm-fact (key "/domain/fact/location/R-1") (value "C-BS"))

		; encode the whole predicate with all arguments as path. This solves
		; the issues mentioned before, but leads to a rather unusual syntax.
		; (wm-fact (key "/domain/fact/person-in-front/Peter") (value TRUE))
		; (wm-fact (key "/domain/fact/location/R-1/C-BS") (value TRUE))

		; The following still encodes the full URL, but further follows
	  ; more of a URL-style encoding, with the arguments separated by a
	  ; question mark and lists split by ampersands (may require encoding of
	  ; the ampersand if it has to appear in the values itself).
		;(wm-fact (key "/domain/fact/person-in-front?Peter") (value TRUE))
		;(wm-fact (key "/domain/fact/location?R-1&C-BS") (value TRUE))

		; It could even allow for named predicate assignment for
	  ; a predicate (location ?robot ?place)
		; To be tested whether this is too verbose or has performance implications
		(wm-fact (id "/domain/fact/person-in-front?name=Peter") (value TRUE))
		; the following will trigger an error about non-unique ID
		;(wm-fact (id "/domain/fact/person-in-front?name=Peter") (value FALSE))
		(wm-fact (id "/domain/fact/location?robot=R-1&place=C-BS") (value TRUE))
		; Conversion tests
		; ID -> key
		; the next two cannot be domain facts, they cannot deal with multi-valued args
		(wm-fact (id "/wm/loaded?robot=R-1&objects=foo,bar") (value TRUE))
		(wm-fact (id "/wm/loaded?objects=foo2,bar2&robot=R-2") (value TRUE))
		(wm-fact (id "/domain/fact/holding?object=some-obj&robot=R-2") (value TRUE))
		; key -> ID
		(wm-fact (key wm loaded args? robot R-1 objects [ cup plate ] ) (value TRUE))
		(wm-fact (key wm loaded-order args? objects [ cup plate ] robot R-1) (value TRUE))
		(wm-fact (key domain fact loctest args? robot R-1 place X ) (value TRUE))
		(wm-fact (key wm broken args? robot R-1 objects [ cup plate ) (value TRUE))
	)
)


(defrule state-robot-location
	;(wm-fact (key "/domain/fact/location?R-1&C-BS") (value TRUE))
	(wm-fact (key domain fact location args? robot ?r place ?p) (value TRUE))
	=>
	(printout error "Robot " ?r " is at " ?p crlf)
)
