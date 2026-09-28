# ePlanSys. Epistemic Planning System for ROS2

![ePlanSys Logo](eplansys_docs/eplansys.png)

[![epistemic-planner (humble)](https://github.com/ePlanSys/eplansys/actions/workflows/epistemic-humble.yaml/badge.svg?branch=rolling)](https://github.com/ePlanSys/eplansys/actions/workflows/epistemic-humble.yaml)
[![rolling](https://github.com/ePlanSys/eplansys/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/ePlanSys/eplansys/actions/workflows/rolling.yaml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue)](https://docs.ros.org)
[![IεPC 2026](https://img.shields.io/badge/IεPC_2026-Intermediate-purple)](https://sites.google.com/view/epistemic-competition/)

ROS2 Epistemic Planning System (**eplansys** in short) is a project whose
objective is to provide Robotics developers with a reliable, simple, and
efficient EPDDL-based planning system with Dynamic Epistemic Logic reasoning.
It is implemented in ROS2 and built on top of
[PlanSys2](https://github.com/PlanSys2/ros2_planning_system).

This project is the result of research on epistemic planning for autonomous
multi-robot systems. PlanSys2 greatly inspired this project. In addition to
epistemic extensions, we contribute key new aspects: S5/KD45 world models,
EPDDL domain support, and heuristic search over epistemic states.

We hope that this software helps to include formal epistemic reasoning in
more Robotics projects, offering a practical bridge between DEL theory and
ROS2 deployment.

An epistemic problem is written in EPDDL — a domain and a problem file, the
epistemic counterparts of a PDDL domain and problem — and handed to the system
the way classical PlanSys2 is handed a `.pddl` domain. Grounding those sources
into the Kripke model the planner searches over is done by
[plank](https://github.com/HanielUlises/plank), the EPDDL toolkit by
Alessandro Burigana and Francesco Fabiano, which `plansys2_epddl_grounder`
builds and runs for you.

## Trying it

The image builds the workspace on ROS 2 Humble, with the planner and the
grounder at the commits this release pins, and runs the two-site survey: two
scouts, two sites, and a policy that branches on what each one finds.

```bash
docker build -t eplansys .
docker run --rm eplansys
docker run --rm eplansys ros2 launch eplansys_demo survey_sites_launch.py north:=clean parallel:=true
```

The mission prints its elapsed time and exits when the goal holds.

## Installing

Each release carries Debian packages for ROS 2 Humble on Ubuntu 22.04, and the
release itself is the apt repository:

```bash
echo "deb [trusted=yes] https://github.com/ePlanSys/eplansys/releases/download/v0.3.0 ./" |
  sudo tee /etc/apt/sources.list.d/eplansys.list
sudo apt update
sudo apt install ros-humble-eplansys ros-humble-eplansys-demo
source /opt/ros/humble/setup.bash
ros2 launch eplansys_demo survey_sites_launch.py
```

The PlanSys2 packages in it are this repository's fork, released under the
upstream names at 3.0.0, and replace the upstream 2.0.x packages when both are
available.

We want to invite you to contribute to this Open Source project!

**Documentation: [eplansys.github.io/eplansys](https://eplansys.github.io/eplansys)**

**Visit the [PlanSys2 Web Page](https://plansys2.github.io) for tutorials and background.**
