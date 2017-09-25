(deftemplate testtempl
  (slot name (type SYMBOL))
  (slot someval (type FLOAT))
)
(defrule test-rule
  ?tf <- (testfact)
=>
  (assert (test-rule-finished))
  (assert (testtempl (name foo) (someval 4.2)))
)

(defrule success-check
  (test-rule-finished)
=>
  (assert (test-succeeded))
)
