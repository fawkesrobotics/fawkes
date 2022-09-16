(defrule execute-effect-pick-up
  ?p<-(plan-action (action-name pick-up) 
               (param-values ?x ?r $?) (state SENSED-EFFECTS-WAIT))
  ?f<- (domain-fact (name ontable) (param-values ?x )) 
  ?g<- (domain-fact (name clear) (param-values ?x) )
  
  ?h<- (domain-fact (name handempty) (param-values ?r) )
=>
  (printout t "Executes effects pick up - block: " ?x " robot: " ?r crlf crlf)
  (retract ?f)  
  (retract ?g)  
  (retract ?h)
  (assert (domain-fact (name handfull) (param-values ?r)) )
  (assert (domain-fact (name holding) (param-values ?x )))
  ;(modify ?p (state EXECUTION-SUCCEEDED))
)

(defrule execute-effect-stack
  (plan-action (action-name stack) (param-values ?x ?y ?r $?) (state SENSED-EFFECTS-WAIT))
  ?f<- (domain-fact (name holding) (param-values ?x)) 
  ?g<- (domain-fact (name clear) (param-values ?y))  
  ?h<- (domain-fact (name handfull) (param-values ?r))
=>
  (printout t "Executes effects stack - block: " ?x " on " ?y " robot: " ?r crlf crlf)
  (retract ?f)  
  (retract ?g)  
  (retract ?h)
  (assert (domain-fact (name clear) (param-values ?x)))
  (assert (domain-fact (name handempty) (param-values ?r )))
  (assert (domain-fact (name on) (param-values ?x ?y)))
)

(defrule execute-effect-unstack
  (plan-action (action-name unstack) (param-values ?x ?y ?r $?) (state SENSED-EFFECTS-WAIT))
  ?f<- (domain-fact (name on) (param-values ?x ?y)) 
  ?g<- (domain-fact (name clear) (param-values ?x))  
  ?h<- (domain-fact (name handempty) (param-values ?r))
=>
  (printout t "Executes effects unstack - block: " ?x " off " ?y " robot: " ?r crlf crlf)
  (retract ?f)  
  (retract ?g)  
  (retract ?h)
  (assert (domain-fact (name clear) (param-values ?y)))
  (assert (domain-fact (name holding) (param-values ?x )))
  (assert (domain-fact (name handfull) (param-values ?r)))
)

(defrule execute-effect-putdown
  (plan-action (action-name put-down) (param-values ?x ?r $?) (state SENSED-EFFECTS-WAIT))
  ?f<- (domain-fact (name holding) (param-values ?x)) 
  ?g<- (domain-fact (name handfull) (param-values ?r))  
=>
  (printout t "Executes effects put-down - block: " ?x " robot: " ?r crlf crlf)
  (retract ?f)  
  (retract ?g)  
  (assert (domain-fact (name clear) (param-values ?x)))
  (assert (domain-fact (name handempty) (param-values ?r )))
  (assert (domain-fact (name ontable) (param-values ?x)))
)
