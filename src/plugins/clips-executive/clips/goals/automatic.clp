
;---------------------------------------------------------------------------
;  automatic.clp - CLIPS executive - automatic advancing sub-goal
;
;  Created: Tue Jun 12 23:12:15 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule automatic-subgoal-expand
	?g <- (goal (class AUTOMATIC-SUBGOAL) (mode SELECTED))
	=>
	(modify ?g (mode EXPANDED))
)

(defrule automatic-subgoal-evaluate
	?g <- (goal (class AUTOMATIC-SUBGOAL) (mode FINISHED))
	=>
	(modify ?g (mode EVALUATED))
)
