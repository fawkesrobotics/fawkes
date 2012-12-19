
;---------------------------------------------------------------------------
;  init.clp - CLIPS agent initialization file
;
;  Created: Sat Jun 16 12:34:54 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*CLIPS_DIRS* = (get-clips-dirs)
  ?*DEBUG* = 1  ;debug levels: 0 ~ none, 1 ~ minimal, 2 ~ more, 3 ~ maximum
  ?*CONFIG_PREFIX* = "/clips-agent"
)

(deffunction resolve-file (?file)
  (foreach ?d ?*CLIPS_DIRS*
	   (bind ?fn (str-cat ?d ?file))
	   (printout t "Testing" ?fn crlf)
	   (if (open ?fn file-clips-tmp)
	    then
	     (close file-clips-tmp)
	     (return ?fn)
	   )
  )
  (return FALSE)
)

(load* (resolve-file utils.clp))
(load* (resolve-file time.clp))
(load* (resolve-file config.clp))


(defrule load-config
  (init)
  =>
  (load-config ?*CONFIG_PREFIX*)
)

(defrule enable-debug
  (init)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value ?v))
  =>
  (if (eq ?v true) then
    (printout t "CLIPS Agent: enabling debugging" crlf)
    (watch facts)
    (watch rules)
  )
)

;(dribble-on "trace.txt")


(reset)
(run)
