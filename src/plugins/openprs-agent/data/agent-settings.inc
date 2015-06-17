
;---------------------------------------------------------------------------
;  agent-settings.inc - OpenPRS agent settings loader
;
;  Created: Mon Sep 08 15:54:39 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

include "new-default.inc"

; debug tracing configuration
trace receive off
trace intend off

; Settings for the various features
declare predicate setting
declare predicate agent-settings-load

include "config.inc"

load opf "agent-settings.opf"

add (agent-settings-load)
