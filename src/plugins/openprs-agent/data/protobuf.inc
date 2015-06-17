
;---------------------------------------------------------------------------
;  protobuf.opf - OpenPRS protobuf access
;
;  Created: Wed Sep 03 14:39:39 2014
;  Copyright  2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

declare predicate protobuf-initialized
declare predicate protobuf-server-client-connected
declare predicate protobuf-server-client-disconnected
declare predicate protobuf-client-connected
declare predicate protobuf-client-disconnected
declare ff protobuf-msg 1

load external "mod_protobuf" "init"
load opf "protobuf.opf"
