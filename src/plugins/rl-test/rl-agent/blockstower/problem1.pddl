(define (problem blocksTower)
    (:domain blocksTower)
    (:objects 
        a - block
        b - block
        c - block
        d - block
        e - block
        robot - robot
    )
    (:init 
        (clear a) 
        (clear b) 
        (clear c) 
        (clear d) 
        (clear e) 
        (ontable a) 
        (ontable b)
        (ontable c) 
        (ontable d) 
        (ontable e) 
        (handempty robot)

        ; action literals
        (tower-c1 a c) 
        (tower-c2 b d e)
        ; (tower-c1 a b)
        ; (tower-c1 b a)
        ; (tower-c1 a c)
        ; (tower-c1 c a)
        ; (tower-c1 a d)
        ; (tower-c1 d a)
        ; (tower-c1 a e)
        ; (tower-c1 e a)
        ; (tower-c1 b c)
        ; (tower-c1 c b)
        ; (tower-c1 b d)
        ; (tower-c1 d b)
        ; (tower-c1 b e)
        ; (tower-c1 e b)
        ; (tower-c1 c d)
        ; (tower-c1 d c)
        ; (tower-c1 c e)
        ; (tower-c1 e c)
        ; (tower-c1 d e)
        ; (tower-c1 e d)


        ; (tower-c2 a b c)
        ; (tower-c2 b c a)
        ; (tower-c2 c a b)
        
        ; (tower-c2 a b d)
        ; (tower-c2 b d a)
        ; (tower-c2 d a b)
        
        ; (tower-c2 a b e)
        ; (tower-c2 b e a)
        ; (tower-c2 e a b)
        
        ; (tower-c2 a d c)
        ; (tower-c2 d c a)
        ; (tower-c2 c a d)
        
        ; (tower-c2 a e c)
        ; (tower-c2 e c a)
        ; (tower-c2 c a e)
        
        ; (tower-c2 d b c)
        ; (tower-c2 b c d)
        ; (tower-c2 c d b)
        
        ; (tower-c2 e b c)
        ; (tower-c2 b c e)
        ; (tower-c2 c e b)

    )
    ;(:goal (and (on d c) (on c b) (on b a)))
    (:goal (and (tower-c1-finished a c) (tower-c2-finished b d e)))
)
