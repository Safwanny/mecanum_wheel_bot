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

import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _prepend_paths(existing, additions):
    paths = additions + existing.split(os.pathsep)
    return os.pathsep.join(dict.fromkeys(path for path in paths if path))


def _without_path_prefix(existing, blocked_prefixes):
    paths = existing.split(os.pathsep)
    return os.pathsep.join(
        path for path in paths
        if path and not any(
            path == prefix or path.startswith(prefix + os.sep)
            for prefix in blocked_prefixes
        )
    )


def _workspace_controller_overlay_prefixes():
    prefix = get_package_prefix('mecanum_drive_controller')
    if prefix.startswith('/opt/ros/'):
        return []
    return [prefix]


def _gazebo_environment():
    ros_prefix = get_package_prefix('gz_sim_vendor')
    blocked_controller_prefixes = _workspace_controller_overlay_prefixes()
    vendor_names = (
        'gz_sim_vendor',
        'gz_sensors_vendor',
        'gz_physics_vendor',
        'gz_gui_vendor',
        'gz_transport_vendor',
        'gz_rendering_vendor',
        'gz_plugin_vendor',
        'gz_fuel_tools_vendor',
        'gz_msgs_vendor',
        'gz_common_vendor',
        'gz_ogre_next_vendor',
        'gz_dartsim_vendor',
        'sdformat_vendor',
        'gz_math_vendor',
        'gz_utils_vendor',
        'gz_tools_vendor',
        'gz_cmake_vendor',
    )

    library_paths = [
        os.path.join(ros_prefix, 'opt', name, 'lib') for name in vendor_names
    ]
    library_paths.extend(
        [
            os.path.join(ros_prefix, 'lib', 'x86_64-linux-gnu'),
            os.path.join(ros_prefix, 'lib'),
        ]
    )
    executable_paths = [
        os.path.join(ros_prefix, 'opt', name, 'bin') for name in vendor_names
    ]
    config_paths = [
        os.path.join(ros_prefix, 'opt', name, 'share', 'gz')
        for name in vendor_names
    ]

    return [
        SetEnvironmentVariable(
            'AMENT_PREFIX_PATH',
            _without_path_prefix(
                os.environ.get('AMENT_PREFIX_PATH', ''),
                blocked_controller_prefixes,
            ),
        ),
        SetEnvironmentVariable(
            'CMAKE_PREFIX_PATH',
            _without_path_prefix(
                os.environ.get('CMAKE_PREFIX_PATH', ''),
                blocked_controller_prefixes,
            ),
        ),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            _prepend_paths(
                _without_path_prefix(
                    os.environ.get('LD_LIBRARY_PATH', ''),
                    blocked_controller_prefixes,
                ),
                library_paths,
            ),
        ),
        SetEnvironmentVariable(
            'PATH',
            _prepend_paths(os.environ.get('PATH', ''), executable_paths),
        ),
        SetEnvironmentVariable(
            'GZ_CONFIG_PATH',
            _prepend_paths(os.environ.get('GZ_CONFIG_PATH', ''), config_paths),
        ),
    ]


def _resolve_world(context, worlds_directory):
    collision_model = LaunchConfiguration(
        'roller_collision_model').perform(context)
    if collision_model not in ('cylinder', 'barrel'):
        raise RuntimeError(
            'roller_collision_model must be cylinder or barrel')
    requested = LaunchConfiguration('world').perform(context)
    requested_path = Path(requested).expanduser()
    if requested_path.is_absolute():
        resolved = requested_path
    else:
        filename = requested if requested.endswith('.sdf') else requested + '.sdf'
        resolved = Path(worlds_directory) / filename
    if not resolved.is_file():
        raise RuntimeError(
            f"Gazebo world '{requested}' resolved to missing file: {resolved}"
        )
    render_engine = LaunchConfiguration('render_engine').perform(context)
    if render_engine not in ('ogre', 'ogre2'):
        raise RuntimeError("render_engine must be 'ogre' or 'ogre2'")
    max_step_text = LaunchConfiguration(
        'physics_max_step_size').perform(context)
    try:
        max_step_size = float(max_step_text)
    except ValueError as error:
        raise RuntimeError(
            'physics_max_step_size must be a positive finite number'
        ) from error
    if not math.isfinite(max_step_size) or max_step_size <= 0.0:
        raise RuntimeError(
            'physics_max_step_size must be a positive finite number')

    tree = ET.parse(resolved)
    physics = tree.findall('./world/physics')
    if len(physics) != 1:
        raise RuntimeError(
            f'Expected one physics element in {resolved}, found '
            f'{len(physics)}')
    max_step = physics[0].find('max_step_size')
    if max_step is None:
        max_step = ET.SubElement(physics[0], 'max_step_size')
    max_step.text = max_step_text

    if render_engine != 'ogre2':
        sensors_plugins = [
            plugin for plugin in tree.findall('.//plugin')
            if plugin.attrib.get('name') == 'gz::sim::systems::Sensors'
        ]
        if len(sensors_plugins) != 1:
            raise RuntimeError(
                f'Expected one Gazebo Sensors plugin in {resolved}, found '
                f'{len(sensors_plugins)}'
            )
        engine = sensors_plugins[0].find('render_engine')
        if engine is None:
            engine = ET.SubElement(sensors_plugins[0], 'render_engine')
        engine.text = render_engine
    generated = Path(
        f'/tmp/mobile_base_world_{os.getpid()}_{render_engine}.sdf'
    )
    tree.write(generated, encoding='utf-8', xml_declaration=True)
    resolved = generated
    context.launch_configurations['resolved_world'] = str(resolved.resolve())
    return []


def generate_launch_description():
    description_share = FindPackageShare('mobile_base_description')
    bringup_share = FindPackageShare('mobile_base_bringup')

    model = PathJoinSubstitution(
        [description_share, 'urdf', 'mobile_base.urdf.xacro']
    )
    controllers = PathJoinSubstitution(
        [bringup_share, 'config', 'controllers.yaml']
    )
    rviz_config = PathJoinSubstitution(
        [description_share, 'rviz', 'mobile_base_sim.rviz']
    )
    gazebo_worlds = os.path.join(
        get_package_share_directory('mobile_base_gazebo'), 'worlds'
    )
    world = LaunchConfiguration('resolved_world')
    simulation_sdf = '/tmp/mobile_base_sim_' + str(os.getpid()) + '.sdf'
    sdf_generator = PathJoinSubstitution(
        [FindPackagePrefix('mobile_base_bringup'), 'lib', 'mobile_base_bringup',
         'generate_sim_sdf.py']
    )
    gz_launch = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                model,
                ' use_gazebo:=true controllers_file:=',
                controllers,
                ' roller_joint_damping:=',
                LaunchConfiguration('roller_joint_damping'),
                ' roller_joint_friction:=',
                LaunchConfiguration('roller_joint_friction'),
                ' roller_contact_mu:=',
                LaunchConfiguration('roller_contact_mu'),
                ' roller_collision_model:=',
                LaunchConfiguration('roller_collision_model'),
                ' front_left_roller_phase:=',
                LaunchConfiguration('front_left_roller_phase'),
                ' front_right_roller_phase:=',
                LaunchConfiguration('front_right_roller_phase'),
                ' rear_right_roller_phase:=',
                LaunchConfiguration('rear_right_roller_phase'),
                ' rear_left_roller_phase:=',
                LaunchConfiguration('rear_left_roller_phase'),
            ]
        ),
        value_type=str,
    )
    generate_sdf = ExecuteProcess(
        cmd=[
            sdf_generator,
            '--xacro',
            model,
            '--controllers',
            controllers,
            '--output',
            simulation_sdf,
            '--roller-joint-damping',
            LaunchConfiguration('roller_joint_damping'),
            '--roller-joint-friction',
            LaunchConfiguration('roller_joint_friction'),
            '--roller-contact-mu',
            LaunchConfiguration('roller_contact_mu'),
            '--roller-collision-model',
            LaunchConfiguration('roller_collision_model'),
            '--front-left-roller-phase',
            LaunchConfiguration('front_left_roller_phase'),
            '--front-right-roller-phase',
            LaunchConfiguration('front_right_roller_phase'),
            '--rear-right-roller-phase',
            LaunchConfiguration('rear_right_roller_phase'),
            '--rear-left-roller-phase',
            LaunchConfiguration('rear_left_roller_phase'),
        ],
        output='screen',
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        output='screen',
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file',
            simulation_sdf,
            '-name',
            'mobile_base',
            '-allow_renaming',
            'true',
            '-x',
            LaunchConfiguration('x'),
            '-y',
            LaunchConfiguration('y'),
            '-z',
            LaunchConfiguration('z'),
            '-Y',
            LaunchConfiguration('yaw'),
        ],
        output='screen',
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            'controller_manager',
            '--switch-timeout',
            '20.0',
            '--service-call-timeout',
            '20.0',
        ],
        output='screen',
    )
    base_controller_spawner_localized = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mobile_base_controller',
            '--controller-manager',
            'controller_manager',
            '--switch-timeout',
            '20.0',
            '--service-call-timeout',
            '20.0',
            '--controller-ros-args=-r',
            '--controller-ros-args=mobile_base_controller/tf_odometry:=/tf',
            '--controller-ros-args=-p',
            '--controller-ros-args=enable_odom_tf:=false',
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('start_controller'), "'.lower() == "
            "'true' and '", LaunchConfiguration('localization'),
            "'.lower() == 'true'",
        ])),
        output='screen',
    )
    base_controller_spawner_raw = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mobile_base_controller',
            '--controller-manager',
            'controller_manager',
            '--switch-timeout',
            '20.0',
            '--service-call-timeout',
            '20.0',
            '--controller-ros-args=-r',
            '--controller-ros-args=mobile_base_controller/tf_odometry:=/tf',
            '--controller-ros-args=-p',
            '--controller-ros-args=enable_odom_tf:=true',
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('start_controller'), "'.lower() == "
            "'true' and '", LaunchConfiguration('localization'),
            "'.lower() != 'true'",
        ])),
        output='screen',
    )
    raw_trajectory = Node(
        package='mobile_base_tools',
        executable='odom_to_path',
        name='raw_odometry_to_path',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    filtered_trajectory = Node(
        package='mobile_base_tools',
        executable='odom_to_path',
        name='filtered_odometry_to_path',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_topic': '/odometry/filtered',
            'path_topic': '/mobile_base/filtered_trajectory',
            'reset_service': '/mobile_base/filtered_trajectory/reset',
        }],
        condition=IfCondition(LaunchConfiguration('localization')),
        output='screen',
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mobile_base_localization'),
                'launch',
                'localization.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('localization')),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('simulation_rviz')),
        output='screen',
    )

    def after_robot_spawn(event, context):
        del context
        if event.returncode == 0:
            return [joint_state_spawner]
        return [
            EmitEvent(event=Shutdown(reason='robot spawn failed')),
        ]

    def after_sdf_generation(event, context):
        del context
        if event.returncode == 0:
            return [spawn_robot]
        return [
            EmitEvent(event=Shutdown(reason='simulation SDF generation failed')),
        ]

    def after_joint_state_spawner(event, context):
        del context
        if event.returncode == 0:
            return [
                base_controller_spawner_localized,
                base_controller_spawner_raw,
                localization,
                raw_trajectory,
                filtered_trajectory,
                rviz,
            ]
        return [
            EmitEvent(event=Shutdown(reason='joint-state controller failed')),
        ]

    return LaunchDescription(
        [
            *_gazebo_environment(),
            DeclareLaunchArgument(
                'world',
                default_value='empty',
                description=(
                    'Installed world name, with or without .sdf, or an '
                    'absolute SDF path.'
                ),
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use the Gazebo simulation clock for ROS nodes.',
            ),
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='true'),
            # Controller spawners start RViz from a delayed process-exit
            # handler. Capture this include's value now so a later include
            # cannot overwrite it through the shared launch context.
            SetLaunchConfiguration(
                'simulation_rviz', LaunchConfiguration('rviz')
            ),
            DeclareLaunchArgument(
                'render_engine',
                default_value='ogre2',
                description=(
                    "Gazebo sensor render engine: 'ogre2' normally or "
                    "'ogre' for Mesa software-rendered validation."
                ),
            ),
            DeclareLaunchArgument(
                'physics_max_step_size',
                default_value='0.001',
                description='Gazebo physics integration step in seconds.',
            ),
            DeclareLaunchArgument(
                'localization',
                default_value='true',
                description=(
                    'Run robot_localization and give it sole ownership of '
                    'odom to base_footprint TF.'
                ),
            ),
            DeclareLaunchArgument(
                'start_controller',
                default_value='true',
                description='Start the mecanum ros2_control controller.',
            ),
            DeclareLaunchArgument(
                'roller_joint_damping', default_value='0.0'),
            DeclareLaunchArgument(
                'roller_joint_friction', default_value='0.0'),
            DeclareLaunchArgument(
                'roller_contact_mu', default_value='1.0'),
            DeclareLaunchArgument(
                'roller_collision_model', default_value='barrel'),
            DeclareLaunchArgument(
                'front_left_roller_phase', default_value='0.22193969'),
            DeclareLaunchArgument(
                'front_right_roller_phase', default_value='0.48030419'),
            DeclareLaunchArgument(
                'rear_right_roller_phase', default_value='0.19668582'),
            DeclareLaunchArgument(
                'rear_left_roller_phase', default_value='0.24790784'),
            DeclareLaunchArgument('x', default_value='0.0'),
            DeclareLaunchArgument('y', default_value='0.0'),
            DeclareLaunchArgument('z', default_value='0.08'),
            DeclareLaunchArgument('yaw', default_value='0.0'),
            OpaqueFunction(
                function=_resolve_world,
                args=[gazebo_worlds],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={'gz_args': ['-r ', world]}.items(),
                condition=IfCondition(LaunchConfiguration('gui')),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={'gz_args': ['-r -s ', world]}.items(),
                condition=UnlessCondition(LaunchConfiguration('gui')),
            ),
            robot_state_publisher,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=generate_sdf,
                    on_exit=after_sdf_generation,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=after_robot_spawn,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=after_joint_state_spawner,
                )
            ),
            generate_sdf,
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                ],
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration(
                            'use_sim_time'
                        )
                    }
                ],
                output='screen',
            ),
        ]
    )
