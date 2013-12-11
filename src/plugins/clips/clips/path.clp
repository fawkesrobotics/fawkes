
;---------------------------------------------------------------------------
;  path.clp - search path utils
;
;  Created: Sat Jun 16 15:45:57 2012 (Mexico City)
;  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defglobal
  ?*FILE-SEARCH-PATH*   = (create$)
  ?*FF-PATH-SUBST-WHAT* = (create$)
  ?*FF-PATH-SUBST-BY*   = (create$)
)

; Completely reset path information.
; This will reset the search path and substitution patterns.
; By default they will sustain a (reset).
(deffunction path-reset ()
  (build (str-cat "(defglobal ?*FILE-SEARCH-PATH*   = (create$)" crlf
		  "           ?*FF-PATH-SUBST-WHAT* = (create$)" crlf
		  "           ?*FF-PATH-SUBST-BY*   = (create$))"))
)

(deffunction path-subst (?path)
  (bind ?rv ?path)
  (foreach ?s ?*FF-PATH-SUBST-WHAT*
    (bind ?rv (str-replace ?rv ?s (nth$ ?s-index ?*FF-PATH-SUBST-BY*)))
  )
  (return ?rv)
)

(deffunction path-get ()
  (return ?*FILE-SEARCH-PATH*)
)


(deffunction path-add-subst (?what ?by)
  (bind ?new-what (append$ ?*FF-PATH-SUBST-WHAT* ?what))
  (bind ?new-by   (append$ ?*FF-PATH-SUBST-BY*   ?by))

  (build (str-cat "(defglobal ?*FF-PATH-SUBST-WHAT* = (create$ " (implode$ ?new-what) "))"))
  (build (str-cat "(defglobal ?*FF-PATH-SUBST-BY*   = (create$ " (implode$ ?new-by) "))"))
)

(deffunction path-add ($?path)
  (foreach ?p ?path
    (bind ?new-path (append$ ?*FILE-SEARCH-PATH* (path-subst ?p)))
    (build (str-cat "(defglobal ?*FILE-SEARCH-PATH* = (create$ " (implode$ ?new-path) "))"))
  )
)

(deffunction path-resolve (?file)
  (foreach ?d ?*FILE-SEARCH-PATH*
    (bind ?fn (str-cat ?d ?file))
    (printout t "Testing file " ?fn crlf)
    (if (open ?fn file-clips-tmp)
     then
      (close file-clips-tmp)
      (return ?fn)
    )
  )
  (return FALSE)
)

(deffunction path-load (?file)
  (load* (path-resolve ?file))
)
