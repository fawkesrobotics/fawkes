
;---------------------------------------------------------------------------
;  agent-init.inc - OpenPRS agent initialization file
;
;  Created: Tue Aug 26 11:38:32 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

include "new-default.inc"

; Settings for the various features
declare predicate setting
add (setting "idle-looptime-sec" 0)
add (setting "idle-looptime-usec" 25000)
add (setting "bb-auto-read" T)
add (setting "bb-auto-cleanup" T)

; debug tracing configuration
trace receive off
trace intend off


; Including feature files and load external modules
include "meta-coalesce.inc"
include "time.inc"
include "blackboard.inc"
include "protobuf.inc"

load external "mod_skiller" "init"

; declare and load agent-specific elements
declare op_predicate say
declare predicate agent-init

add (agent-init)

load opf "agent.opf"
