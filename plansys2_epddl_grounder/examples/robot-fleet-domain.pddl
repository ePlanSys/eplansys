(define (domain robot-fleet)

;; The classical half of the same mission.
;;
;; The EPDDL domain next to this file says what the robots come to know; this
;; says what they physically do. The executor never reads the EPDDL — it takes
;; the action name out of a plan item, looks it up here, and runs the behavior
;; tree registered for it. Both halves have to exist, and the action_mapping in
;; plansys2_epistemic_planner/examples/mappings/robot-fleet.json is what joins
;; them: `inspect_r1` on the epistemic side is `(inspect_corridor r1)` here.
;;
;; The preconditions and effects below are the mechanical ones — where the
;; robot is, whether it has looked. The epistemic conditions that actually
;; drive the plan ("r1 does not yet know whether the corridor is blocked")
;; cannot be written in PDDL at all, which is the reason the other file exists.

(:requirements :strips :typing :adl :durative-actions)

(:types
robot
waypoint
)

(:predicates

(robot_at ?r - robot ?wp - waypoint)
(connected ?wp1 ?wp2 - waypoint)
(is_junction ?wp - waypoint)

;; True once ?r has pointed its camera down the corridor. What it saw is not
;; recorded here — a predicate cannot hold "r1 knows whether", which is the
;; whole point — so this only says the look happened.
(inspected ?r - robot)

;; True once ?r has transmitted. Again, not what it transmitted.
(reported ?r - robot)

)

(:functions
)

(:durative-action goto_junction
    :parameters (?r - robot ?from ?to - waypoint)
    :duration ( = ?duration 30)
    :condition (and
        (at start(robot_at ?r ?from))
        (at start(connected ?from ?to))
        (at start(is_junction ?to))
    )
    :effect (and
        (at start(not(robot_at ?r ?from)))
        (at end(robot_at ?r ?to))
    )
)

(:durative-action inspect_corridor
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_at ?r ?wp))
        (at start(is_junction ?wp))
    )
    :effect (and
        (at end(inspected ?r))
    )
)

;; Two actions, not one with an argument, because the plan carries which one to
;; run: the epistemic planner produces a policy that branches on what the
;; camera saw, and the executor dispatches whichever branch execution took.
(:durative-action report_blocked
    :parameters (?r - robot)
    :duration ( = ?duration 1)
    :condition (and
        (at start(inspected ?r))
    )
    :effect (and
        (at end(reported ?r))
    )
)

(:durative-action report_clear
    :parameters (?r - robot)
    :duration ( = ?duration 1)
    :condition (and
        (at start(inspected ?r))
    )
    :effect (and
        (at end(reported ?r))
    )
)

)
