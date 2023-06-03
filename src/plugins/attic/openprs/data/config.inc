
;---------------------------------------------------------------------------
;  config.inc - OpenPRS config access
;
;  Created: Fri Sep 05 13:34:37 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

declare cwp confval
declare cwp config-loaded
declare cwp settings-loaded
declare op_predicate config-load-to-settings
declare op_predicate config-to-settings
declare op_predicate config-cleanup

load external "mod_config" "init"
load opf "config.opf"
