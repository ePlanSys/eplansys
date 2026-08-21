;; The classical half of the depot mission: two corridors instead of one, and
;; an action's parameters say which.
;;
;; As in test_5, nothing here records what anyone knows or whether a corridor
;; is blocked. That lives in the epistemic task; this is only what the executor
;; needs to drive the actions the mapping translates into.

(define (domain depot)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
  corridor
)

(:predicates
  (at_base ?r - robot)
  (at_junction ?r - robot ?c - corridor)
  (inspected ?r - robot ?c - corridor)
  (reported ?r - robot ?c - corridor)
)

(:durative-action goto_junction
  :parameters (?r - robot ?c - corridor)
  :duration (= ?duration 5)
  :condition (and
    (at start (at_base ?r)))
  :effect (and
    (at start (not (at_base ?r)))
    (at end (at_junction ?r ?c)))
)

(:durative-action inspect_corridor
  :parameters (?r - robot ?c - corridor)
  :duration (= ?duration 5)
  :condition (and
    (over all (at_junction ?r ?c)))
  :effect (and
    (at end (inspected ?r ?c)))
)

(:durative-action report_clear
  :parameters (?r - robot ?c - corridor)
  :duration (= ?duration 5)
  :condition (and
    (at start (inspected ?r ?c))
    (over all (at_junction ?r ?c)))
  :effect (and
    (at end (reported ?r ?c)))
)

(:durative-action report_blocked
  :parameters (?r - robot ?c - corridor)
  :duration (= ?duration 5)
  :condition (and
    (at start (inspected ?r ?c))
    (over all (at_junction ?r ?c)))
  :effect (and
    (at end (reported ?r ?c)))
)

)
