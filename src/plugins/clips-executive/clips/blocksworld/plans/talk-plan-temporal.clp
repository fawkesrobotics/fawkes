
(defrule plan-talk-expand-temporal
  ?g <- (goal (mode SELECTED) (id ?goal-id) (class TALK))
  =>
  (bind ?plan-id (sym-cat ?goal-id -PLAN))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (type TEMPORAL))
    (plan-action (id 10) (goal-id ?goal-id) (plan-id ?plan-id)
                 (duration 4.0) (dispatch-time 0.0)
                 (action-name say-hello)
                 (param-names name) (param-values peggy))
    (plan-action (id 20) (goal-id ?goal-id) (plan-id ?plan-id)
                 (duration 4.0) (dispatch-time 0.0)
                 (action-name print)
                 (param-names severity text)
                 (param-values warn "This is a print test"))
    (plan-action (id 30) (goal-id ?goal-id) (plan-id ?plan-id)
                 (duration 4.0) (dispatch-time 15.0)
                 (action-name say-goodbye) (param-values peggy))
  )
)
