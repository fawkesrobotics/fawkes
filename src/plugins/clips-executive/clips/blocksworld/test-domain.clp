;---------------------------------------------------------------------------
;  test-domain.clp - Test Domain
;
;  Created: Wed 04 Oct 2017 18:14:57 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defrule load-domain
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "blocksworld/domain.pddl"))
  (assert (domain-loaded))
  (assert (skiller-control))
)

(defrule test-domain-set-sensed-predicates
  (executive-init)
  (domain-loaded)
  ?p <- (domain-predicate (name said) (sensed FALSE))
  ?p1 <- (domain-predicate (name ontable) (sensed FALSE))
  ?p2 <- (domain-predicate (name clear) (sensed FALSE))
  ?p3 <- (domain-predicate (name pickup) (sensed FALSE))
  ?p4 <- (domain-predicate (name handempty) (sensed FALSE))
  ?p5 <- (domain-predicate (name handfull) (sensed FALSE))
  ?p6 <- (domain-predicate (name holding) (sensed FALSE))
=>
  (modify ?p (sensed TRUE))
  (modify ?p1 (sensed TRUE))
  (modify ?p2 (sensed TRUE))
  (modify ?p3 (sensed TRUE))
  (modify ?p4 (sensed TRUE))
  (modify ?p5 (sensed TRUE))
  (modify ?p6 (sensed TRUE))
)

(defrule load-initial-facts
  (executive-init)
  (domain-loaded)
  =>
  (assert
	 (domain-object (name hello) (type text))
	 (domain-object (name goodbye) (type text))
	 (domain-object (name lock1) (type name))
	 (domain-operator (name print) (param-names severity text))
	 (domain-operator (name say-cleanup))
	 (domain-fact (name said) (param-values bob hello))


    ;blocksworld facts
    (domain-object (name a) (type block))
	  (domain-object (name b) (type block))
    (domain-object (name c) (type block))
	  (domain-object (name d) (type block))
	  (domain-object (name e) (type block))
	  (domain-object (name f) (type block))
    (domain-object (name robo1) (type robot))

    (domain-fact (name clear) (param-values a))
    (domain-fact (name clear) (param-values b)) 
    (domain-fact (name clear) (param-values c)) 
    (domain-fact (name clear) (param-values d))
    (domain-fact (name clear) (param-values e))  
    (domain-fact (name clear) (param-values f))  

    (domain-fact (name ontable) (param-values a)) 
    (domain-fact (name ontable) (param-values b)) 
    (domain-fact (name ontable) (param-values c)) 
    (domain-fact (name ontable) (param-values d)) 
    (domain-fact (name ontable) (param-values e)) 
    (domain-fact (name ontable) (param-values f)) 
    
    (domain-fact (name handempty) (param-values robo1)) 
        
    ; action literals
    (domain-fact (name pickup) (param-values a))
    (domain-fact (name pickup) (param-values b)) 
    (domain-fact (name pickup) (param-values c)) 
    (domain-fact (name pickup) (param-values d)) 
    (domain-fact (name pickup) (param-values e)) 
    (domain-fact (name pickup) (param-values f)) 

    (domain-fact (name putdown) (param-values a))
    (domain-fact (name putdown) (param-values b)) 
    (domain-fact (name putdown) (param-values c)) 
    (domain-fact (name putdown) (param-values d)) 
    (domain-fact (name putdown) (param-values e)) 
    (domain-fact (name putdown) (param-values f)) 

    (domain-fact (name unstack) (param-values a))
    (domain-fact (name unstack) (param-values b)) 
    (domain-fact (name unstack) (param-values c)) 
    (domain-fact (name unstack) (param-values d)) 
    (domain-fact (name unstack) (param-values e)) 
    (domain-fact (name unstack) (param-values f)) 

    (domain-fact (name stack) (param-values a b))
    (domain-fact (name stack) (param-values a c)) 
    (domain-fact (name stack) (param-values a d)) 
    (domain-fact (name stack) (param-values a e)) 
    (domain-fact (name stack) (param-values a f)) 

    (domain-fact (name stack) (param-values b a))
    (domain-fact (name stack) (param-values b c)) 
    (domain-fact (name stack) (param-values b d)) 
    (domain-fact (name stack) (param-values b e)) 
    (domain-fact (name stack) (param-values b f))

    (domain-fact (name stack) (param-values c a))
    (domain-fact (name stack) (param-values c b)) 
    (domain-fact (name stack) (param-values c d)) 
    (domain-fact (name stack) (param-values c e)) 
    (domain-fact (name stack) (param-values c f)) 

    (domain-fact (name stack) (param-values d a))
    (domain-fact (name stack) (param-values d b)) 
    (domain-fact (name stack) (param-values d c)) 
    (domain-fact (name stack) (param-values d e))  
    (domain-fact (name stack) (param-values d f))  


    (domain-fact (name stack) (param-values e a))
    (domain-fact (name stack) (param-values e b)) 
    (domain-fact (name stack) (param-values e c)) 
    (domain-fact (name stack) (param-values e d))
    (domain-fact (name stack) (param-values e f))


    (domain-fact (name stack) (param-values f a))
    (domain-fact (name stack) (param-values f b)) 
    (domain-fact (name stack) (param-values f c)) 
    (domain-fact (name stack) (param-values f d))
    (domain-fact (name stack) (param-values f e))
	 (domain-facts-loaded)
	)
)

(defrule delete-domain-facts
  (reset-domain-facts)
  ?f <- (domain-fact)
  =>
	;(printout t crlf "delete domain fact " ?f crlf crlf)
  (retract ?f)  
)

(defrule delete-goals
  (reset-domain-facts)
  ?g <- (goal)
  =>
	;(printout t crlf "delete goal " ?g crlf crlf)
  (retract ?g)  
)

(defrule reset-domain
  ?r<-(reset-domain-facts)
  (not (domain-fact))
  (not (goal))
  ?d<- (domain-loaded)
	?fl<-(domain-facts-loaded)
  ?g <-(goals-loaded)
  =>
	(printout t crlf "reset domain running" crlf crlf)
  (retract ?d)
  (retract ?fl)
  (retract ?r)
  (retract ?g)
  (assert (reset-domain-running))
)

(defrule reset-running
  ?r <-(reset-domain-running)
  (domain-loaded)
	(domain-facts-loaded)
  (goals-loaded)
  =>
	(printout t crlf "reset domain finish" crlf crlf)
  (retract ?r)
  (assert (reset-domain-finish))
)
                                                                            
                                                                                
; (defrule apply-pick-up-action                                                           
;   "Pseudo-execute action by changing its state to FINAL."                       
;   ?aa <- (apply-action ?goal-id ?plan-id ?action-id)                            
;   ?pa <- (plan-action (id ?action-id) (action-name pick-up) (goal-id ?goal-id) (plan-id ?plan-id))    
; => 
;   (printout t "Test apply pick up action" crlf)                                                                             
;   (modify ?pa (state EXECUTION-SUCCEEDED))                                      
;   (retract ?aa)                                                                 
; ) 


(defrule test-domain-set-domain-fact-said-hello
  (plan-action (action-name say-hello|say-hello-again) (param-values peggy) (state SENSED-EFFECTS-WAIT))
=>
  (assert (domain-fact (name said) (param-values peggy hello)))
)

(defrule test-domain-set-domain-fact-said-goodbye
  (plan-action (action-name say-goodbye) (state SENSED-EFFECTS-WAIT))
=>
  (assert (domain-fact (name said) (param-values peggy goodbye)))
)

