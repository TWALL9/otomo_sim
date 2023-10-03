import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    package_name ='otomo_sim'
    pkg_share_dir = get_package_share_directory(package_name)

    # include state publisher, which also launches URDF
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('otomo_control'),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('otomo_core'),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # twist_mux_params = os.path.join(pkg_share_dir,'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package='twist_mux',
    #         executable='twist_mux',
    #         parameters=[twist_mux_params, {'use_sim_time': True}],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )

    gazebo_models_path = os.path.join(pkg_share_dir, 'worlds', 'models')
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path
    gazebo_world_file = os.path.join(pkg_share_dir, 'worlds', 'apartment.world')

    gazebo_params_file = os.path.join(pkg_share_dir, 'config', 'gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': gazebo_world_file, 'extra_gazebo_args': '--ros-args --remap /laser_controller/out:=scan --params-file ' + gazebo_params_file}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen'
    )

    controllers = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('otomo_control'),'launch','controllers.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        # twist_mux,
        gazebo,
        spawn_entity,
        controllers
    ])
