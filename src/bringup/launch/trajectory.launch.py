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

from jsonschema import ValidationError, validate

import json
import shutil
import logging


# schema = {
#     "type": "object",
#     "properties": {
#         "name": {"type": "string"},
#         "iiwa_ros2_folder": {"type": "string"},
#         "monitor_joints": {"type": "boolean"},
#         "iiwa": {
#             "type": "object",
#             "properties": {
#                 "launch_file": {"type": "string"},
#                 "controllers_file": {"type": "string"},
#                 "params": {"type": "object"},
#             },
#             "required": ["launch_file"],
#         },
#         "trajectory_node": {
#             "type": "object",
#             "properties": {
#                 "prefix": {"type": "string"},
#             },
#         },
#     },
#     "required": ["iiwa", "controller_optimizer"],
# }


# def load_json_file(json_file_path) -> dict:
#     config: dict = {}
#     try:
#         with open(json_file_path, "r") as f:
#             config = json.load(f)
#     except Exception as e:
#         print("Error loading config file: ", e)
#         raise Exception("Could not load config file")

#     try:
#         validate(instance=config, schema=schema)
#     except ValidationError as e:
#         raise ValueError(f"Invalid config file: {e}")

#     return config


def launch_setup(context, *args, **kwargs):
    logger = logging.getLogger("launch")

    # config_path = LaunchConfiguration("config_path").perform(context)
    # config: dict = load_json_file(config_path)

    # Controller file cant be redirected without compiling for avoiding this I copy the file
    # directly

    trajectory_node = Node(
        package="trajectory_node",
        executable="talker",
        name="trajectory_node",
        output="screen",
        emulate_tty=True,
        prefix=["valgrind -q --vgdb-error=0"],
    )

    logger.info(f"Trajectory node initialized")

    # quit_all_nodes_when_controller_has_ended = RegisterEventHandler(
    #     event_handler=OnExecutionComplete(
    #         target_action=controller_, on_completion=[Shutdown()]
    #     )
    # )

    nodes = [
        trajectory_node,
        # quit_all_nodes_when_controller_has_ended,
    ]

    return nodes


def generate_launch_description():
    # workaround for getting the config path on the launch file

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
