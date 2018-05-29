;---------------------------------------------------------------------------
;  saliences.clp - CLIPS Executive Salience Values
;
;  Created: Thu Sep 21 13:09:19 2017
;  Copyright  2012-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; CLIPS Executive Salience Register
; If you need a different salience, use one of the following values.
; Refrain from adding more unless absolutely necessary.
; Avoid saliences for two-way inter-area synchronization like:
; - Rule area A: assert (something)
; - Rule area B: trigger on (something) with higher salience
; - Rule area A: remove (something) with normal or lower salience
; This leads to confusing interactions. Saliences should only shield areas
; completely and enforce some one-way chain, for example:
; - Rule area A: assert (something)
; - Rule area B: trigger on (something), assert (this-thing) with
;                higher salience
; - Rule area C: trigger on (this-thing) (but avoid firing before area B is
;                done, and this "before" cannot be encoded more nicely in
;                a direct dependency.
; Example could be domain model update -> wm sync -> goal reasoner (and
; avoid to make a goal decision before the wm is updated).
(defglobal
  ?*SALIENCE-FIRST*         =  10000
  ?*SALIENCE-INIT*          =   9010
  ?*SALIENCE-INIT-LATE*     =   9000
  ?*SALIENCE-WM-IDKEY*      =   5200
  ?*SALIENCE-WM-SYNC-DEL*   =   5100
  ?*SALIENCE-WM-SYNC-ADD*   =   5000
  ?*SALIENCE-DOMAIN-GROUND* =   4200
  ?*SALIENCE-DOMAIN-CHECK*  =   4100
  ?*SALIENCE-DOMAIN-APPLY*  =   4000
  ?*SALIENCE-HIGH*          =   1000
  ?*SALIENCE-MODERATE*      =    500
	; the following is just informative
  ;?*SALIENCE-NORMAL*       =      0
  ?*SALIENCE-LOW*           =  -1000
  ?*SALIENCE-LAST*          = -10000
)
