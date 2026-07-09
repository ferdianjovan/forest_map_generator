#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg = FindPackageShare("forest_map_generator")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_configuration_file",
                default_value=PathJoinSubstitution(
                    [this_pkg, "configs", "map_configuration.yaml"]
                ),
                description="Path to the map-generation YAML configuration file.",
            ),
            DeclareLaunchArgument(
                "output_world_file",
                default_value="world_with_trees.world",
                description="Generated world file name or path.",
            ),
            Node(
                package="forest_map_generator",
                executable="forest_map_generator",
                name="forest_map_generator",
                output="screen",
                parameters=[
                    {
                        "map_configuration_file": LaunchConfiguration(
                            "map_configuration_file"
                        ),
                        "output_world_file": LaunchConfiguration("output_world_file"),
                    }
                ],
            ),
        ]
    )
