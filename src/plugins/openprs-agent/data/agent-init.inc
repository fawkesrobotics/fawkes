
;---------------------------------------------------------------------------
;  agent-init.inc - OpenPRS agent initialization file
;
;  Created: Tue Aug 26 11:38:32 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

include "new-default.inc"

declare predicate setting

include "meta-coalesce.inc"
include "time.inc"

load external "mod_skiller" "init"

declare op_predicate say
declare predicate agent-init

add (setting "idle-looptime-sec" 0)
add (setting "idle-looptime-usec" 25000)
add (setting "busy-looptime-sec" 0)
add (setting "busy-looptime-usec" 5000)
add (agent-init)

load opf "agent.opf"

trace receive off
trace intend off
