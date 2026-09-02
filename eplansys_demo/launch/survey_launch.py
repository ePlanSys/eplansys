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
The site survey: three robots, and a mission about who may know what.

Unlike the corridor, this one is stated in EPDDL and ground by plank at start
up, which is the ordinary way to pose an epistemic problem. It needs `plank`
built and on PATH.

    ros2 launch eplansys_demo survey_launch.py
    ros2 launch eplansys_demo survey_launch.py site:=clean

The goal has three conjuncts and each rules out a different shortcut: the scout
must find out, the relay must come to know that the scout knows, and the
observer must not come to know at all. The third is what makes the open channel
unusable and the mission interesting.
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


OUTCOMES = {
    'dirty': 'e-scan-dirty',
    'clean': 'e-scan-clean',
}


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('eplansys_demo')

    domain = os.path.join(pkg, 'epddl', 'survey-team.epddl')
    problem = os.path.join(pkg, 'epddl', 'survey-team-problem.epddl')
    mapping = os.path.join(pkg, 'pddl', 'survey-mapping.json')
    model = os.path.join(pkg, 'pddl', 'survey.pddl')

    site = LaunchConfiguration('site').perform(context)
    if site not in OUTCOMES:
        raise RuntimeError(
            f'site:={site} is not one of {sorted(OUTCOMES)}. It is what the '
            'scout turns out to find.')

    with open(os.path.join(pkg, 'params', 'survey.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('MAPPING_FILE', mapping))

    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_survey.yaml', delete=False)
    filled.write(params)
    filled.close()

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': model,
            'params_file': filled.name,
            'epistemic_state': 'True',
        }.items())

    actions = Node(
        package='eplansys_demo',
        executable='survey_actions',
        name='survey_actions',
        output='screen',
        arguments=['--outcome', OUTCOMES[site]])

    mission = Node(
        package='eplansys_demo',
        executable='survey_mission',
        name='survey_mission',
        output='screen')

    finish = RegisterEventHandler(
        OnProcessExit(target_action=mission, on_exit=[EmitEvent(event=Shutdown())]))

    return [plansys2, actions, mission, finish]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'site', default_value='dirty',
            description='What the scout turns out to find: dirty or clean.'),
        OpaqueFunction(function=launch_setup),
    ])
