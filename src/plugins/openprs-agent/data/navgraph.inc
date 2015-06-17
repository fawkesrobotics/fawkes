
;---------------------------------------------------------------------------
;  navgraph.inc - OpenPRS navgraph access
;
;  Created: Fri Sep 05 16:41:40 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

declare cwp navgraph
declare cwp navgraph-node
declare cwp navgraph-edge

declare op_predicate navgraph-cleanup
declare op_predicate navgraph-reload

load external "mod_navgraph" "init"
load opf "navgraph.opf"
