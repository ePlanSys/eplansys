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
The two-site survey: two scouts, two sites and nothing shared between them.

    ros2 launch eplansys_demo survey_sites_launch.py
    ros2 launch eplansys_demo survey_sites_launch.py north:=clean south:=dirty

Stated in EPDDL and ground by plank at start up, so `plank` has to be built and
on PATH.

The mission is the one-site survey run twice in different hands. Each scout
carries the instrument its own site needs, so neither half can be done by the
other's robot, and no action of one half appears in any precondition of the
other. The policy that comes back therefore holds two independent halves, and
the wall clock the mission prints says whether they were dispatched as such.

    ros2 launch eplansys_demo survey_sites_launch.py parallel:=true

turns the dispatch of those runs on. It is the same policy either way: what
changes is how much of it is in flight at once, and the elapsed time the
mission prints at the end.
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
    'north': {'dirty': 'e-scan-north-dirty', 'clean': 'e-scan-north-clean'},
    'south': {'dirty': 'e-scan-south-dirty', 'clean': 'e-scan-south-clean'},
}


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('eplansys_demo')

    domain = os.path.join(pkg, 'epddl', 'survey-sites.epddl')
    problem = os.path.join(pkg, 'epddl', 'survey-sites-problem.epddl')
    mapping = os.path.join(pkg, 'pddl', 'survey-sites-mapping.json')
    model = os.path.join(pkg, 'pddl', 'survey-sites.pddl')

    found = {}
    for site in OUTCOMES:
        choice = LaunchConfiguration(site).perform(context)
        if choice not in OUTCOMES[site]:
            raise RuntimeError(
                f'{site}:={choice} is not one of {sorted(OUTCOMES[site])}. It is '
                'what that site turns out to hold.')
        found[site] = OUTCOMES[site][choice]

    parallel = LaunchConfiguration('parallel').perform(context).lower()
    if parallel not in ('true', 'false'):
        raise RuntimeError(
            f'parallel:={parallel} is neither true nor false. It says whether the '
            'two halves of the policy are dispatched together.')

    with open(os.path.join(pkg, 'params', 'survey_sites.yaml')) as handle:
        params = handle.read()
    params = (params
              .replace('EPDDL_DOMAIN', domain)
              .replace('EPDDL_PROBLEM', problem)
              .replace('MAPPING_FILE', mapping)
              .replace('PARALLEL_DISPATCH', parallel))

    filled = tempfile.NamedTemporaryFile(
        mode='w', suffix='_survey_sites.yaml', delete=False)
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
        executable='survey_sites_actions',
        name='survey_sites_actions',
        output='screen',
        arguments=['--north', found['north'], '--south', found['south']])

    mission = Node(
        package='eplansys_demo',
        executable='survey_sites_mission',
        name='survey_sites_mission',
        output='screen')

    finish = RegisterEventHandler(
        OnProcessExit(target_action=mission, on_exit=[EmitEvent(event=Shutdown())]))

    return [plansys2, actions, mission, finish]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'north', default_value='dirty',
            description='What the north site turns out to hold: dirty or clean.'),
        DeclareLaunchArgument(
            'south', default_value='clean',
            description='What the south site turns out to hold: dirty or clean.'),
        DeclareLaunchArgument(
            'parallel', default_value='false',
            description='Dispatch the policy\'s independent runs together.'),
        OpaqueFunction(function=launch_setup),
    ])
