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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    # The regions this node watches are named under `epistemic_perception:` in
    # the parameters file, and they have to be the regions the domain talks
    # about: a region whose atom is not in the loaded task resolves to a
    # formula the state cannot parse, and says so on the first observation.
    epistemic_perception_cmd = Node(
        package='plansys2_epistemic_perception',
        executable='epistemic_perception_node',
        name='epistemic_perception',
        namespace=namespace,
        output='screen',
        parameters=[params_file])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    ld.add_action(epistemic_perception_cmd)

    return ld
