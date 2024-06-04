"""
ROS 2 launching file
"""

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete

import logging


def launch_setup(context, *args, **kwargs):
    logger = logging.getLogger("launch")

    nn_controller = Node(
        package="NN_controller",
        executable="nn_controller",
        name="nn_controller",
        output="screen",
        emulate_tty=True,
        prefix=[""],
    )

    logger.info(f"NN controller initialized")

    quit_all_nodes_when_controller_has_ended = RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=nn_controller, on_completion=[Shutdown()]
        )
    )

    nodes = [
        nn_controller,
        quit_all_nodes_when_controller_has_ended,
    ]

    return nodes


def generate_launch_description():
    # workaround for getting the config path on the launch file

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
