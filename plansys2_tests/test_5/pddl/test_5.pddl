;; The classical half of the corridor mission.
;;
;; What the robots know lives in the epistemic task the planner solved; this
;; file is what the executor needs to drive the actions that task names: the
;; PlanSys2 expressions the action mapping translates into, with durations and
;; with preconditions and effects over plain facts.
;;
;; Deliberately thin. The mission's difficulty is epistemic — nobody knows
;; whether the corridor is blocked — and putting a fact about that here would
;; be recording knowledge as a predicate, which is the thing the epistemic
;; state exists to avoid.

(define (domain fleet)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
)

(:predicates
  (at_base ?r - robot)
  (at_junction ?r - robot)
  (inspected ?r - robot)
  (reported ?r - robot)
)

(:durative-action goto_junction
  :parameters (?r - robot)
  :duration (= ?duration 5)
  :condition (and
    (at start (at_base ?r)))
  :effect (and
    (at start (not (at_base ?r)))
    (at end (at_junction ?r)))
)

(:durative-action inspect_corridor
  :parameters (?r - robot)
  :duration (= ?duration 5)
  :condition (and
    (over all (at_junction ?r)))
  :effect (and
    (at end (inspected ?r)))
)

;; The two announcements. Which of them runs is the whole question the policy
;; branches on, and from the PDDL side they are indistinguishable: both say
;; the robot has reported.
(:durative-action report_clear
  :parameters (?r - robot)
  :duration (= ?duration 5)
  :condition (and
    (at start (inspected ?r))
    (over all (at_junction ?r)))
  :effect (and
    (at end (reported ?r)))
)

(:durative-action report_blocked
  :parameters (?r - robot)
  :duration (= ?duration 5)
  :condition (and
    (at start (inspected ?r))
    (over all (at_junction ?r)))
  :effect (and
    (at end (reported ?r)))
)

)
