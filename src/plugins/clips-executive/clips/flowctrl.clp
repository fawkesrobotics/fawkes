
;---------------------------------------------------------------------------
;  flowctrl.clp - CLIPS executive flow control
;
;  Created: Thu Sep 21 11:47:02 2017
;  Copyright  2012-2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defmodule FLOWCTRL
	(import MAIN ?ALL)
	(import PLAN ?ALL)
	(import PLAN-EXEC ?ALL)
	(import SKILL-EXEC ?ALL)
)


(defrule FLOWCTRL::init
	(declare (salience ?*PRIORITY-INIT*) (auto-focus TRUE))
	(executive-init)
	=>
	(printout t "Flow initialized" crlf)
	(assert (module-initialized FLOWCTRL))
)

(defrule FLOWCTRL::flow-start
	(declare (auto-focus TRUE))
	(start)
	=>
	(printout t "Flow control starting up" crlf)
	(focus SPEC PLAN-EXEC SKILL-EXEC FLOWCTRL)
)
