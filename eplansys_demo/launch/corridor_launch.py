# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The corridor mission, in one command.

Two robots and a corridor nobody has looked down. The goal is that both come to
know whether it is blocked, which no classical planner can even state: there is
no fact about the corridor to plan towards, only a state of knowledge to reach.

The solution is therefore a policy and not a sequence. r1 drives out, looks,
and reports what it found --- and which report it makes is decided while the
mission runs, by what the looking turned up.

    ros2 launch eplansys_demo corridor_launch.py
    ros2 launch eplansys_demo corridor_launch.py corridor:=blocked

Run it both ways. The plan is the same; the execution is not.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# What the robot turns out to see, by the name of the event it observes. These
# are the two outcomes the inspect action's model defines; the policy has a
# branch for each.
OUTCOMES = {
    'clear': 'e-inspect-clear',
    'blocked': 'e-inspect-blocked',
}


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('eplansys_demo')

    task = os.path.join(pkg, 'epddl', 'corridor-task.json')
    mapping = os.path.join(pkg, 'pddl', 'corridor-mapping.json')
    model = os.path.join(pkg, 'pddl', 'corridor.pddl')

    corridor = LaunchConfiguration('corridor').perform(context)
    if corridor not in OUTCOMES:
        raise RuntimeError(
            f"corridor:={corridor} is not one of {sorted(OUTCOMES)}. It is what "
            'the robot turns out to see when it looks.')

    # The parameters name the task by placeholder, because its path belongs to
    # whoever installed the package.
    with open(os.path.join(pkg, 'params', 'corridor.yaml')) as handle:
        params = handle.read()
    params = params.replace('TASK_FILE', task).replace('MAPPING_FILE', mapping)

    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_corridor.yaml', delete=False)
    filled.write(params)
    filled.close()

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': model,
            'params_file': filled.name,
            # The fifth managed node: what the robots know. Without it the
            # policy's knowledge guards have nothing to ask.
            'epistemic_state': 'True',
        }.items())

    actions = Node(
        package='eplansys_demo',
        executable='corridor_actions',
        name='corridor_actions',
        output='screen',
        # On the command line and not as a parameter: the four performers share
        # one process, and a parameter set here would reach only the node this
        # block names.
        arguments=['--outcome', OUTCOMES[corridor]])

    mission = Node(
        package='eplansys_demo',
        executable='corridor_mission',
        name='corridor_mission',
        output='screen')

    # The mission is the demo. When it ends, so does everything it needed:
    # a demo left running after its point has been made is one the reader has
    # to know how to stop.
    finish = RegisterEventHandler(
        OnProcessExit(target_action=mission, on_exit=[EmitEvent(event=Shutdown())]))

    return [plansys2, actions, mission, finish]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'corridor', default_value='clear',
            description='What the robot turns out to see: clear or blocked. '
                        'The plan does not depend on it; the execution does.'),
        OpaqueFunction(function=launch_setup),
    ])
