
;---------------------------------------------------------------------------
;  blackboard.opf - OpenPRS blackboard access
;
;  Created: Tue Sep 02 15:47:09 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

declare predicate blackboard-interface
declare predicate bb-data

; Constants for opening mode
declare symbol BB-READ
declare symbol BB-WRITE

load external "mod_blackboard" "init"
load opf "blackboard.opf"
