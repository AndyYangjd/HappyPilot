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
            name='joy_avp_demo_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='joy_avp_demo',
                    plugin='joypilot::joy_avp_demo::JoyAvpDemoNode',
                    name='joy_avp_demo_node'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
