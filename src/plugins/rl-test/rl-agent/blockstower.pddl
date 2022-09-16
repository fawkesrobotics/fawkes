;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Mapping blocks goals to rl actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain blocksTower)
    (:requirements :strips :typing)
    (:types block robot)
    (:predicates 
        (on ?x - block ?y - block)
        (ontable ?x - block)
        (clear ?x - block)
        (handempty ?x - robot)
        (handfull ?x - robot)
        (holding ?x - block)

        (tower-c1 ?buttom - block ?top - block)
        (tower-c2 ?buttom - block ?middle - block ?top - block)

        (tower-c1-finished ?buttom - block ?top - block)
        (tower-c2-finished ?buttom - block ?middle - block ?top - block)
        ;(pickup ?x - block)
        ;(putdown ?x - block)
        ;(stack ?x - block ?y - block)
        ;(unstack ?x - block)
    )

    ; (:actions tower-c1 tower-c2)

    (:action tower-c1
        :parameters (?buttom - block ?top - block ?robot - robot)
        :precondition (and
            (tower-c1 ?buttom ?top)
            (clear ?buttom)
            (clear ?top) 
            (ontable ?buttom)            
            (ontable ?top) 
            (handempty ?robot)
        )
        :effect (and
            (tower-c1-finished ?buttom ?top)
            (ontable ?buttom)
            (not (ontable ?top))
            (not (clear ?buttom))
            (clear ?top)
            (on ?top ?buttom)
            (handempty ?robot)
        )
    )


    (:action tower-c2
        :parameters (?buttom - block ?middle - block ?top - block ?robot - robot)
        :precondition (and
            (tower-c2 ?buttom ?middle ?top)
            (clear ?buttom)
            (clear ?middle)
            (clear ?top) 
            (ontable ?buttom)            
            (ontable ?middle)            
            (ontable ?top) 
            (handempty ?robot)
        )
        :effect (and
            (tower-c2-finished ?buttom ?middle ?top)
            (ontable ?buttom)
            (not (ontable ?middle))
            (not (ontable ?top))
            (not (clear ?buttom))
            (not (clear ?middle))
            (clear ?top)
            (on ?top ?middle)
            (on ?middle ?buttom)
            (handempty ?robot)
        )
    )


   
)
