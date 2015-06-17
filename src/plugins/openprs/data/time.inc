;---------------------------------------------------------------------------
;  time.inc - OpenPRS time utils
;
;  Created: Mon Sep 01 16:00:33 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

load external "mod_time" "init"

;add (setting "print-time" T)
undeclare function time
declare ff time 0
declare predicate time-update
declare be fawkes-time
declare ff timer 1

add (time 0 0)
load opf "time.opf"
