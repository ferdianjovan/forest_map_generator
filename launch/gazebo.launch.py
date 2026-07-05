from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable


def generate_launch_description():
    this_pkg = FindPackageShare('forest_map_generator')
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([
        ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world_name = LaunchConfiguration('world_name', default="world_with_trees.world")
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value=use_sim_time,
        description='If true, use simulated clock.'
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value=world_name,
        description="Name of the world file to launch",
    )
    gazebo_process = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([this_pkg, 'worlds', world_name])],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                PathJoinSubstitution([this_pkg, 'models'])
            ),
            use_sim_time_arg,
            world_name_arg,
            gazebo_process,
        ]
    )
