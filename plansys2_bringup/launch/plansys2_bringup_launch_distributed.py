# Copyright 2019 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def lifecycle_manager(context, *args, **kwargs):
    """
    Build the lifecycle manager for the set of nodes that was actually started.

    Naming a node that is not running makes startup block and then fail, so the
    managed set has to match the started set exactly. This used to be two
    managers with two written-out lists, one per value of a single flag. A
    second optional node turns that into four, and only three of the four are
    systems anyone can run, so the list is assembled here instead: an
    OpaqueFunction runs once the launch arguments have values, which is what
    lets the set be decided in Python rather than out of substitutions.
    """
    started = ['domain_expert', 'problem_expert', 'planner', 'executor']

    with_state = IfCondition(LaunchConfiguration('epistemic_state')).evaluate(context)
    with_perception = IfCondition(LaunchConfiguration('epistemic_perception')).evaluate(context)

    if with_perception and not with_state:
        raise RuntimeError(
            'epistemic_perception:=True needs epistemic_state:=True. Perception reports '
            'what it reads to the epistemic state, over that node\'s services, and there '
            'would be none to report to.')

    if with_state:
        started.append('epistemic_state')
    if with_perception:
        started.append('epistemic_perception')

    return [Node(
        package='plansys2_lifecycle_manager',
        executable='lifecycle_manager_node',
        name='lifecycle_manager_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{'managed_nodes': started}])]


def generate_launch_description():
    bringup_dir = get_package_share_directory('plansys2_bringup')

    # Create the launch configuration variables
    model_file = LaunchConfiguration('model_file')
    problem_file = LaunchConfiguration('problem_file')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    action_bt_file = LaunchConfiguration('action_bt_file')
    start_action_bt_file = LaunchConfiguration('start_action_bt_file')
    end_action_bt_file = LaunchConfiguration('end_action_bt_file')
    bt_builder_plugin = LaunchConfiguration('bt_builder_plugin')
    epistemic_state = LaunchConfiguration('epistemic_state')
    epistemic_perception = LaunchConfiguration('epistemic_perception')

    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        description='PDDL Model file')

    declare_problem_file_cmd = DeclareLaunchArgument(
        'problem_file',
        description='PDDL Problem file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'plansys2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_action_bt_file_cmd = DeclareLaunchArgument(
        'action_bt_file',
        default_value=os.path.join(
            get_package_share_directory('plansys2_executor'),
            'behavior_trees', 'plansys2_action_bt.xml'),
        description='BT representing a PDDL action')

    declare_start_action_bt_file_cmd = DeclareLaunchArgument(
        'start_action_bt_file',
        default_value=os.path.join(
            get_package_share_directory('plansys2_executor'),
            'behavior_trees', 'plansys2_start_action_bt.xml'),
        description='BT representing a PDDL start action')

    declare_end_action_bt_file_cmd = DeclareLaunchArgument(
        'end_action_bt_file',
        default_value=os.path.join(
            get_package_share_directory('plansys2_executor'),
            'behavior_trees', 'plansys2_end_action_bt.xml'),
        description='BT representing a PDDL end action')

    declare_bt_builder_plugin_cmd = DeclareLaunchArgument(
        'bt_builder_plugin',
        default_value='SimpleBTBuilder',
        description='Behavior tree builder plugin.',
    )

    declare_epistemic_state_cmd = DeclareLaunchArgument(
        'epistemic_state',
        default_value='False',
        description='Start the epistemic state as a fifth managed node. '
                    'Executing an epistemic policy needs it; a classical '
                    'system has no use for it.',
    )

    declare_epistemic_perception_cmd = DeclareLaunchArgument(
        'epistemic_perception',
        default_value='False',
        description='Start epistemic perception as a sixth managed node, '
                    'which reads named regions of an occupancy grid and '
                    'reports them to the epistemic state. Needs '
                    'epistemic_state:=True, and the regions it watches are '
                    'named under epistemic_perception: in the parameters '
                    'file.',
    )

    domain_expert_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_domain_expert'),
            'launch',
            'domain_expert_launch.py')),
        launch_arguments={
            'model_file': model_file,
            'namespace': namespace,
            'params_file': params_file
        }.items())

    problem_expert_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_problem_expert'),
            'launch',
            'problem_expert_launch.py')),
        launch_arguments={
            'model_file': model_file,
            'problem_file': problem_file,
            'namespace': namespace,
            'params_file': params_file
        }.items())

    planner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_planner'),
            'launch',
            'planner_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'params_file': params_file
        }.items())

    executor_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_executor'),
            'launch',
            'executor_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'params_file': params_file,
            'default_action_bt_xml_filename': action_bt_file,
            'default_start_action_bt_xml_filename': start_action_bt_file,
            'default_end_action_bt_xml_filename': end_action_bt_file,
            'bt_builder_plugin': bt_builder_plugin,
        }.items())

    epistemic_state_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_epistemic_executor'),
            'launch',
            'epistemic_state_launch.py')),
        condition=IfCondition(epistemic_state),
        launch_arguments={
            'namespace': namespace,
            'params_file': params_file
        }.items())

    epistemic_perception_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_epistemic_perception'),
            'launch',
            'epistemic_perception_launch.py')),
        condition=IfCondition(epistemic_perception),
        launch_arguments={
            'namespace': namespace,
            'params_file': params_file
        }.items())

    lifecycle_manager_cmd = OpaqueFunction(function=lifecycle_manager)

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_problem_file_cmd)
    ld.add_action(declare_action_bt_file_cmd)
    ld.add_action(declare_start_action_bt_file_cmd)
    ld.add_action(declare_end_action_bt_file_cmd)
    ld.add_action(declare_bt_builder_plugin_cmd)
    ld.add_action(declare_epistemic_state_cmd)
    ld.add_action(declare_epistemic_perception_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)

    # Declare the launch options
    ld.add_action(domain_expert_cmd)
    ld.add_action(problem_expert_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(executor_cmd)
    ld.add_action(epistemic_state_cmd)
    ld.add_action(epistemic_perception_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
