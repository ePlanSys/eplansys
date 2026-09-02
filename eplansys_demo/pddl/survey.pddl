;; The classical half of the site survey.
;;
;; What the robots know lives in the epistemic task the planner solved; this
;; file is what the executor needs to drive the actions that task names.
;;
;; Deliberately thin, and deliberately blind. Nothing here distinguishes a
;; broadcast from an encrypted relay: both are a robot talking, they take the
;; same time and change the same facts, and a PDDL domain has no vocabulary
;; for who was listening. That difference is the entire mission, and it is
;; expressible only in the epistemic half.

(define (domain survey)
(:requirements :strips :typing :adl :durative-actions)

(:types
  robot
)

(:predicates
  (at_depot ?r - robot)
  (on_site ?r - robot)
  (scanned ?r - robot)
  (told ?r - robot)
)

(:durative-action goto_site
  :parameters (?r - robot)
  :duration (= ?duration 4)
  :condition (and
    (at start (at_depot ?r)))
  :effect (and
    (at start (not (at_depot ?r)))
    (at end (on_site ?r)))
)

(:durative-action scan
  :parameters (?r - robot)
  :duration (= ?duration 3)
  :condition (and
    (over all (on_site ?r)))
  :effect (and
    (at end (scanned ?r)))
)

;; One action for the open channel and one for the team link. They differ in
;; nothing a classical planner can see; the executor drives them the same way.
(:durative-action broadcast
  :parameters (?r - robot)
  :duration (= ?duration 2)
  :condition (and
    (at start (scanned ?r)))
  :effect (and
    (at end (told ?r)))
)

(:durative-action relay
  :parameters (?from ?to - robot)
  :duration (= ?duration 2)
  :condition (and
    (at start (scanned ?from)))
  :effect (and
    (at end (told ?from)))
)
)
