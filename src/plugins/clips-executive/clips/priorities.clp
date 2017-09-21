;---------------------------------------------------------------------------
;  priorities.clp - CLIPS Executive Priority Values
;
;  Created: Thu Sep 21 13:09:19 2017
;  Copyright  2012-2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defmodule EXECUTIVE-PRIORITIES (export ?ALL))

(defglobal EXECUTIVE-PRIORITIES
  ?*PRIORITY-FIRST*   =  1000
  ?*PRIORITY-INIT*    =   600
  ?*PRIORITY-HIGH*    =   500
  ?*PRIORITY-LOW*     =  -100
  ?*PRIORITY-LAST*    =  -500
)
