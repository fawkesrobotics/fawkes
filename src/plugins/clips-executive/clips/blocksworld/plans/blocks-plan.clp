(defrule plan-tower-c1-expand
	?g <- (goal (mode SELECTED) (id ?goal-id) (class TOWER-C1)
				(params buttom ?b
						top ?t))
	?r <- (domain-object (name robo1) (type robot))
	;?c <- (domain-object (name c) (type block))
	;?c <- (wm-fact (key domain object block) (values c))
	=>
	(printout t "Plan expand: robot " ?r " buttom " ?b " top " ?t crlf crlf)
	(bind ?plan-id (sym-cat ?goal-id -PLAN))
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
		
		(plan-action (id 10) (goal-id ?goal-id) (plan-id ?plan-id)
                 	 (duration 4.0)
		             (action-name pick-up)
					 (param-values (create$ ?t robo1)))
		
		(plan-action (id 20) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0)
		             (action-name stack)
					 (param-values (create$ ?t ?b robo1)))
 	)
)

(defrule plan-tower-c2-expand
	?g <- (goal (mode SELECTED) (id ?goal-id) (class TOWER-C2)
				(params buttom ?buttom middle ?middle top ?top))
	?r <- (domain-object (name robo1) (type robot))
	;?c <- (domain-object (name c) (type block))
	;?c <- (wm-fact (key domain object block) (values c))
	=>
	(printout t "Plan expand: robot " ?r " "?buttom " " ?middle " " ?top crlf crlf);" blocks " $?blocks crlf crlf)
	;(bind ?buttom (nth$ 1 $?blocks))
	;(bind ?middle (nth$ 2 $?blocks))
	;(bind ?top (nth$ 3 $?blocks))
	(printout t "Blocks: " ?buttom " m: " ?middle " t: " ?top crlf)
	(bind ?plan-id (sym-cat ?goal-id -PLAN))
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
		
		(plan-action (id 10) (goal-id ?goal-id) (plan-id ?plan-id)
                 	 (duration 4.0)
		             (action-name pick-up)
					 (param-values (create$ ?middle robo1)))
					 ;(param-names x)
		             ;(param-values c )) ;(param-names block)
		
		(plan-action (id 20) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0)
		             (action-name stack)
					 (param-values (create$ ?middle ?buttom robo1)))
		
		(plan-action (id 30) (goal-id ?goal-id) (plan-id ?plan-id)
                 	 (duration 4.0)
		             (action-name pick-up)
					 (param-values (create$ ?top robo1)))
		
		(plan-action (id 40) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0)
		             (action-name stack)
					 (param-values (create$ ?top ?middle robo1)))
	)
)


(defrule plan-blocks-expand
	?g <- (goal (mode SELECTED) (id ?goal-id) (class BLOCKS))
	?r <- (domain-object (name robo1) (type robot))
	?c <- (domain-object (name c) (type block))
	;?c <- (wm-fact (key domain object block) (values c))
	=>
	(printout t "Plan expand: robot " ?r " block " ?c crlf crlf)
	(bind ?plan-id (sym-cat ?goal-id -PLAN))
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
		(plan-action (id 3) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0)
		             (action-name say-hello) (param-values peggy))
		
		; (plan-action (id 10) (goal-id ?goal-id) (plan-id ?plan-id)
        ;          	 (duration 4.0)
		;              (action-name pick-up)
		; 			 (param-values (create$ c robo1)))
		; 			 ;(param-names x)
		;              ;(param-values c )) ;(param-names block)
		
		; (plan-action (id 20) (goal-id ?goal-id) (plan-id ?plan-id)
		;              (duration 4.0)
		;              (action-name stack)
		; 			 (param-values (create$ c d robo1)))
		; 			 ;(param-names x y robot)
		;              ;(param-values c d robo1))
		(plan-action (id 30) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0)
		             (action-name pick-up)
					 (param-values (create$ a robo1)))
		(plan-action (id 40) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0)
		             (action-name stack)
					 (param-values (create$ a c robo1)))
		(plan-action (id 45) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0)
		             (action-name say-cleanup))
		(plan-action (id 50) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0)
		             (action-name unstack)
					 (param-values (create$ a c robo1)))
		(plan-action (id 60) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0)
		             (action-name put-down)
					 (param-values (create$ a robo1)))
	)
)


; (defrule blocks-executable-check
; 	?g <- (goal (mode DISPATCHED) (id ?goal-id) (committed-to ?plan-id) (class BLOCKS) )
; 	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state PENDING) (executable FALSE))
; 	=>
; 	(printout t "goal BLOCKS executable" crlf crlf)
; 	(modify ?pa (executable TRUE))

; )
; (defrule plan-print-expand
; 	?g <- (goal (mode SELECTED) (id ?goal-id) (class PRINT))
; 	=>
; 	(bind ?plan-id (sym-cat ?goal-id -PLAN))
; 	(assert
; 		(plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
; 		(plan-action (id 10) (goal-id ?goal-id) (plan-id ?plan-id)
; 		             (duration 4.0)
; 		             (action-name print)
; 								 (param-names severity text)
; 		             (param-values warn (str-cat ?goal-id " prints something")))
; 		(plan-action (id 20) (goal-id ?goal-id) (plan-id ?plan-id)
; 		             (duration 4.0)
; 		             (action-name print)
; 								 (param-names severity text)
; 		             (param-values warn (str-cat ?goal-id " prints something else")))
; 	)
; )
