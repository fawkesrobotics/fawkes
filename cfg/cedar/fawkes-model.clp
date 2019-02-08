;---------------------------------------------------------------------------
;  fawkes-model.clp - CEDAR example system model for Fawkes
;
;  Created: Mon Dec 16 15:05:57 2013
;  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
;             2011-2013  RWTH Aachen University (KBSG)
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(deffacts cedar-model-fawkes
  (model-fawkes-plugin (name "cedar") (state LOADED))
  (model-fawkes-plugin (name "ros-cmdvel") (state LOADED))
  (model-fawkes-plugin (name "any") (state LOADED) (importance IGNORED))
)
