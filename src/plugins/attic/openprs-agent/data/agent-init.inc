
;---------------------------------------------------------------------------
;  agent-init.inc - OpenPRS agent initialization file
;
;  Created: Tue Aug 26 11:38:32 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; debug tracing configuration
trace receive off
trace intend off
;trace intention failure on
;trace suc_fail on

; Including feature files and load external modules
;include "meta-coalesce.inc"
load external "mod_utils" "init"
include "time.inc"
include "blackboard.inc"
include "protobuf.inc"
include "navgraph.inc"

load external "mod_skiller" "init"

; declare and load agent-specific elements
declare predicate  agent-init

load opf "agent-init.opf"

