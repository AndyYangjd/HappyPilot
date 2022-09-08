#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Example launch file for a new package.
"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with a single component."""
    container = ComposableNodeContainer(
            name='carla_interface_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='carla_interface',
                    plugin='joypilot::carla_interface::CarlaInterfaceNode',
                    name='carla_interface_node'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
