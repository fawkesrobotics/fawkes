(define (problem blocks)
    (:domain blocks)
    (:objects 
        d - block
        b - block
        a - block
        c - block
        e - block
        robot - robot
    )
    (:init
        (clear a)
        (on a b)
        (on b c)
        (on c d)
        (on d e)
        (ontable e)
        (handempty robot)

        ; action literals
        (pickup a)
        (putdown a)
        (unstack a)
        (stack a b)
        (stack a c)
        (stack a d)
        (stack a e)
        (pickup b)
        (putdown b)
        (unstack b)
        (stack b a)
        (stack b c)
        (stack b d)
        (stack b e)
        (pickup c)
        (putdown c)
        (unstack c)
        (stack c b)
        (stack c a)
        (stack c d)
        (stack c e)
        (pickup d)
        (putdown d)
        (unstack d)
        (stack d b)
        (stack d c)
        (stack d a)
        (stack d e)
        (pickup e)
        (putdown e)
        (unstack e)
        (stack e b)
        (stack e c)
        (stack e a)
        (stack e d)

    )
    (:goal (and (on e d) (on d c) (on c b) (on b a)))
)
