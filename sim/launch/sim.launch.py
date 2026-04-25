import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "otomo_sim"
    pkg_share_dir = get_package_share_directory(package_name)

    # include state publisher, which also launches URDF
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("otomo_control"),
                    "launch",
                    "rsp.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )

    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("otomo_control"),
                    "launch",
                    "twist_mux.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("otomo_core"),
                    "launch",
                    "joystick.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    gazebo_models_path = os.path.join(pkg_share_dir, "worlds", "models")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    default_world = os.path.join(pkg_share_dir, "worlds", "obstacles.sdf")

    world = LaunchConfiguration("world")

    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="World to load"
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description", "-name", "otomo_bot", "-z", "0.1"],
        output="screen",
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name), "config", "gazebo_params.yaml"
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"],
    )

    # NOTE: gazebo launches its own copy of controller_manager in the xacro file
    # It also loads the controller parameters from in there. The only thing to launch here are the
    # controllers themselves.

    # controllers = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(
    #             get_package_share_directory('otomo_control'), 'launch', 'controllers.launch.py'
    #         )]),
    #     launch_arguments={'use_sim_time': 'true'}.items()
    # )

    return LaunchDescription(
        [
            rsp,
            twist_mux,
            world_arg,
            gazebo,
            spawn_entity,
            # controllers
            diff_drive_spawner,
            joint_broad_spawner,
            ros_gz_bridge,
            ros_gz_image_bridge,
            joystick,
        ]
    )
