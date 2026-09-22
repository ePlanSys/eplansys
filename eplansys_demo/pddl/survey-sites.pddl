;; The classical half of the two-site survey.
;;
;; As in the one-site mission, what the robots know lives in the epistemic
;; task; this file is what the executor needs to drive the actions that task
;; names, and it is deliberately blind to the difference between the open
;; channel and the team link.
;;
;; The two sites are separate predicates rather than a site parameter, which
;; keeps this file in step with the EPDDL beside it, where a parameter can only
;; be an agent. The durations are what the makespan of a policy is measured
;; from: four seconds to drive, three to scan, two to speak.

(define (domain survey-sites)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
)

(:predicates
  (at_depot ?r - robot)
  (on_north ?r - robot)
  (on_south ?r - robot)
  (scanned_north ?r - robot)
  (scanned_south ?r - robot)
  (told ?r - robot)
)

(:durative-action goto_north
  :parameters (?r - robot)
  :duration (= ?duration 4)
  :condition (and
    (at start (at_depot ?r)))
  :effect (and
    (at end (on_north ?r)))
)

(:durative-action goto_south
  :parameters (?r - robot)
  :duration (= ?duration 4)
  :condition (and
    (at start (at_depot ?r)))
  :effect (and
    (at end (on_south ?r)))
)

(:durative-action scan_north
  :parameters (?r - robot)
  :duration (= ?duration 3)
  :condition (and
    (over all (on_north ?r)))
  :effect (and
    (at end (scanned_north ?r)))
)

(:durative-action scan_south
  :parameters (?r - robot)
  :duration (= ?duration 3)
  :condition (and
    (over all (on_south ?r)))
  :effect (and
    (at end (scanned_south ?r)))
)

;; One pair for the open channel and one for the team link, per site. They
;; differ in nothing a classical planner can see.
(:durative-action broadcast_north
  :parameters (?r - robot)
  :duration (= ?duration 2)
  :condition (and
    (at start (scanned_north ?r)))
  :effect (and
    (at end (told ?r)))
)

(:durative-action broadcast_south
  :parameters (?r - robot)
  :duration (= ?duration 2)
  :condition (and
    (at start (scanned_south ?r)))
  :effect (and
    (at end (told ?r)))
)

(:durative-action relay_north
  :parameters (?from ?to - robot)
  :duration (= ?duration 2)
  :condition (and
    (at start (scanned_north ?from)))
  :effect (and
    (at end (told ?from)))
)

(:durative-action relay_south
  :parameters (?from ?to - robot)
  :duration (= ?duration 2)
  :condition (and
    (at start (scanned_south ?from)))
  :effect (and
    (at end (told ?from)))
)
)
