
;---------------------------------------------------------------------------
;  path.clp - search path utils
;
;  Created: Sat Jun 16 15:45:57 2012 (Mexico City)
;  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defglobal
  ?*FILE-SEARCH-PATH* = (create$)
  ?*FF-PATH-SUBST-WHAT* = (create$)
  ?*FF-PATH-SUBST-BY* = (create$)
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
  (bind ?*FF-PATH-SUBST-WHAT* (append$ ?*FF-PATH-SUBST-WHAT* ?what))
  (bind ?*FF-PATH-SUBST-BY*   (append$ ?*FF-PATH-SUBST-BY*   ?by))
)

(deffunction path-add ($?path)
  (foreach ?p ?path
    (bind ?*FILE-SEARCH-PATH* (append$ ?*FILE-SEARCH-PATH* (path-subst ?p)))
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
