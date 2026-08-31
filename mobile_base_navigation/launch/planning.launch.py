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

"""Launch the simulated robot with full autonomous navigation."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Add navigation to the Phase 2 localization stack.

    The command chain is connected: twist_mux is the single arbiter on
    the controller reference, and the robot drives.
    """
    share = FindPackageShare('mobile_base_navigation')
    localization_launch = PathJoinSubstitution([
        FindPackageShare('mobile_base_bringup'),
        'launch',
        'localization.launch.py',
    ])
    use_sim_time = {'use_sim_time': True}
    # nav2_util's TwistSubscriber and TwistPublisher share this one node-level
    # parameter, so each node is stamped on both sides or neither and every hop
    # in the chain has to agree. The controller's reference interface is
    # TwistStamped, so the whole chain is stamped.
    stamped_cmd_vel = {'enable_stamped_cmd_vel': True}
    bringup_share = FindPackageShare('mobile_base_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Path to a saved Nav2 occupancy-map YAML file.',
        ),
        DeclareLaunchArgument('world', default_value='navigation_basic'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre2'),
        # The costmaps cannot activate until AMCL publishes map -> odom, and
        # AMCL waits for an initial pose. Set autostart:=false to bring the
        # planning nodes up by hand once the particle cloud has converged:
        #   ros2 service call /lifecycle_manager_navigation/manage_nodes \
        #     nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'planner_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'planner.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'local_costmap_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'local_costmap.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'controller_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'controller.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'behavior_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'behavior.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'bt_navigator_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'bt_navigator.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'twist_mux_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'twist_mux.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'estop_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'estop.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'collision_monitor_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'collision_monitor.yaml',
            ]),
        ),
        # This tree tries costmap clearing and waiting before any commanded
        # motion, and its Spin/BackUp distances come from measurement.
        DeclareLaunchArgument(
            'bt_xml',
            default_value=PathJoinSubstitution([
                share, 'behavior_trees',
                'navigate_to_pose_measured_recovery.xml',
            ]),
        ),
        # The smoother's parameters live in mobile_base_bringup so there is a
        # single source of truth for the command-shaping limits.
        DeclareLaunchArgument(
            'velocity_smoother_config',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'velocity_smoother.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                share, 'rviz', 'navigation.rviz',
            ]),
        ),
        # Preserve the wrapper's value before the included stack receives
        # rviz=false. Include launch arguments share the launch context.
        SetLaunchConfiguration(
            'phase3_rviz', LaunchConfiguration('rviz')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
                'render_engine': LaunchConfiguration('render_engine'),
                # This stack runs its own velocity smoother inside the
                # navigation chain, so the bringup one stays off. Two
                # smoothers would both remap onto the controller reference
                # and break the single-arbiter rule.
                'velocity_smoother': 'false',
            }.items(),
        ),
        # map_server is owned by the included localization stack, so it is
        # neither started nor managed here.
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[
                LaunchConfiguration('planner_config'),
                use_sim_time,
            ],
            output='screen',
        ),
        # controller_server owns the local costmap as an embedded sub-node,
        # so there is no standalone costmap node any more. It publishes the
        # velocity command that everything downstream arbitrates and limits.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[
                LaunchConfiguration('controller_config'),
                LaunchConfiguration('local_costmap_config'),
                use_sim_time,
                stamped_cmd_vel,
            ],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            output='screen',
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[
                LaunchConfiguration('behavior_config'),
                use_sim_time,
                stamped_cmd_vel,
            ],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            output='screen',
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[
                LaunchConfiguration('bt_navigator_config'),
                use_sim_time,
                {'default_nav_to_pose_bt_xml': LaunchConfiguration(
                    'bt_xml')},
            ],
            output='screen',
        ),
        # Limits, then the collision gate, then arbitration last.
        #
        # The collision monitor gates AUTONOMY only. That is deliberate: it
        # sits before the mux so a human on the keyboard can always drive out
        # of a corner the monitor is refusing to leave. What the monitor no
        # longer covers, the e-stop does - a twist_mux lock masks every input
        # below its priority, teleop included.
        #
        # Official Nav2 puts the collision monitor last and has no mux;
        # Husarion puts muxing last and has no collision monitor. This is the
        # merge, and the only order where "teleop overrides autonomy" and
        # "e-stop overrides everything" both hold.
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[
                LaunchConfiguration('velocity_smoother_config'),
                use_sim_time,
            ],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            output='screen',
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            parameters=[
                LaunchConfiguration('collision_monitor_config'),
                use_sim_time,
            ],
            output='screen',
        ),
        # Holds the twist_mux lock. It publishes a Bool heartbeat, so if this
        # node dies the lock expires and engages on its own.
        Node(
            package='mobile_base_navigation',
            executable='estop_gate',
            name='estop_gate',
            parameters=[
                LaunchConfiguration('estop_config'),
                use_sim_time,
            ],
            output='screen',
        ),
        # The single arbiter. This is the only node that may publish to the
        # controller's reference interface; everything else feeds it.
        # twist_mux is not a lifecycle node.
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[
                LaunchConfiguration('twist_mux_config'),
                use_sim_time,
            ],
            remappings=[
                ('cmd_vel_out', '/mobile_base_controller/reference'),
            ],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': True,
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'), value_type=bool),
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother',
                    'collision_monitor',
                ],
                # Bond heartbeats are unreliable under sim time: a server
                # activates normally but is never reached by bond, and the
                # manager then reports a spurious bringup failure. 0.0
                # disables the bond check, which is what Nav2 itself does in
                # simulation. The harness, on wall time, keeps 4.0 s.
                'bond_timeout': 0.0,
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='navigation_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('phase3_rviz')),
            output='screen',
        ),
    ])
