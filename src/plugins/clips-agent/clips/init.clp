
;---------------------------------------------------------------------------
;  init.clp - CLIPS agent initialization file
;
;  Created: Sat Jun 16 12:34:54 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*CLIPS_DIRS* = (get-clips-dirs)
  ?*DEBUG* = 2  ;debug levels: 0 ~ none, 1 ~ minimal, 2 ~ more, 3 ~ maximum
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
(load* (resolve-file skills.clp))


(defrule load-config
  (init)
  =>
  (load-config ?*CONFIG_PREFIX*)
)

(defrule load-agent
  (init)
  (confval (path "/clips-agent/agent") (type STRING) (value ?v))
  =>
  (printout t "Loading agent '" ?v "'" crlf)
  (load* (resolve-file (str-cat ?v ".clp")))
)

(defrule enable-debug
  (init)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value ?v))
  =>
  (if (eq ?v true) then
    (printout t "CLIPS debugging enabled, watching facts and rules" crlf)
    (watch facts)
    (watch rules)
    ;(dribble-on "trace.txt")
  )
)

(defrule debug-level
  (init)
  (confval (path "/clips-agent/debug-level") (type UINT) (value ?v))
  =>
  (printout t "Setting debug level to " ?v " (was " ?*DEBUG* ")" crlf)
  (bind ?*DEBUG* ?v)
)

(reset)
(run)
