# Copyright 2026 Safwan
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

"""Show the robot's camera in its own window, alongside a running stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Open a standalone camera viewer.

    Deliberately attaches to whatever is already running rather than starting
    a robot of its own: it is a viewer, and launching a second simulation to
    look at the first one is not what anybody wants. Start the simulation,
    mapping or navigation stack first, then run this in another terminal.

    use_sim_time matters here even though nothing is being controlled. The
    images carry simulation timestamps, and a viewer on wall time treats every
    one of them as far in the past.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Image topic to display.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='True while any Gazebo stack is running.',
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='camera_view',
            # Passed as an argument rather than a remap: rqt_image_view takes
            # the topic positionally and pre-selects it, so the window opens
            # already showing the camera instead of an empty dropdown.
            arguments=[LaunchConfiguration('image_topic')],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
    ])
