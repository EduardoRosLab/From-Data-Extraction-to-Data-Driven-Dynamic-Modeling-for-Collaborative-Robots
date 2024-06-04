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


schema = {
    "type": "object",
    "properties": {
        "name": {"type": "string"},
        "iiwa_ros2_folder": {"type": "string"},
        "monitor_joints": {"type": "boolean"},
        "iiwa": {
            "type": "object",
            "properties": {
                "launch_file": {"type": "string"},
                "controllers_file": {"type": "string"},
                "params": {"type": "object"},
            },
            "required": ["launch_file"],
        },
        "controller_optimizer": {
            "type": "object",
            "properties": {
                "executable": {"type": "string"},
                "prefix": {"type": "string"},
                "params": {"type": "object"},
            },
            "required": ["executable"],
        },
    },
    "required": ["iiwa", "controller_optimizer"],
}


def load_json_file(json_file_path) -> dict:
    config: dict = {}
    try:
        with open(json_file_path, "r") as f:
            config = json.load(f)
    except Exception as e:
        print("Error loading config file: ", e)
        raise Exception("Could not load config file")

    try:
        validate(instance=config, schema=schema)
    except ValidationError as e:
        raise ValueError(f"Invalid config file: {e}")

    return config


def launch_setup(context, *args, **kwargs):
    logger = logging.getLogger("launch")

    config_path = LaunchConfiguration("config_path").perform(context)
    config: dict = load_json_file(config_path)

    iiwa_config = config["iiwa"]

    iiwa_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([iiwa_config["launch_file"]]),
        launch_arguments=iiwa_config.get("params").items()
        if iiwa_config.get("params")
        else [],
    )

    # Controller file cant be redirected without compiling for avoiding this I copy the file
    # directly

    iiwa_folder = config.get("iiwa_ros2_folder", "/ros2_dev/iiwa_ros2")

    iiwa_controller_file = f"{iiwa_folder}/install/iiwa_description/share/iiwa_description/config/iiwa_controllers.yaml"

    if iiwa_config.get("controllers_file"):
        shutil.copyfile(iiwa_config["controllers_file"], iiwa_controller_file)

    logger.info(f"iiwa controller file copied and saved: {iiwa_controller_file}")

    co_config = config["controller_optimizer"]

    controller_optimizer = Node(
        package="controller_optimizer",
        executable=co_config["executable"],
        parameters=[co_config["params"]] if co_config.get("params") else [],
        output="screen",
        prefix=[co_config.get("prefix")] if co_config.get("prefix") else [],
    )

    logger.info(f"Controller optimizer initialized")

    quit_all_nodes_when_controller_has_ended = RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=controller_optimizer,
            on_completion=[Shutdown()],
        )
    )

    nodes = [
        controller_optimizer,
        quit_all_nodes_when_controller_has_ended,
    ]

    return nodes


def generate_launch_description():
    # workaround for getting the config path on the launch file

    return LaunchDescription(
        [DeclareLaunchArgument("config_path"), OpaqueFunction(function=launch_setup)]
    )
