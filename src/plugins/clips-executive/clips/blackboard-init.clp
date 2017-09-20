;load needed interfaces
(defrule blackboard-init
  "Prepare blackboard usage"
  (ff-feature blackboard)
  =>
  (blackboard-enable-time-read)
)
