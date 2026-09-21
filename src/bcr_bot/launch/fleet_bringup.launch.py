#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    bcr_bot_path = get_package_share_directory("bcr_bot")
    multi_spawn_launch = join(bcr_bot_path, "launch", "bcr_bot_multi_spawn.launch.py")

    # Confirmed-safe zone only: x in [-4.6, 4.1], y in [-9.9, 1.9] --
    # entirely inside the double-shelf-walled open floor (ShelfD/E east
    # wall, ShelfF west wall), well clear of the ClutteringA/C/D +
    # Bucket + TrashCan cluster which sits at y > ~2.3. No guessing.
    #
    # robot1 and robot2 share the same x=0 line, driving head-on --
    # genuine conflict, not just shared open floor. robot3 crosses
    # perpendicular through the same midpoint (y=-4).
    robots = [
        {"name": "robot1", "x": "0.0",  "y": "-9.0", "yaw": "1.5708",  "delay": 0.0},
        {"name": "robot2", "x": "0.0",  "y": "1.0",  "yaw": "-1.5708", "delay": 10.0},
        {"name": "robot3", "x": "-3.0", "y": "-4.0", "yaw": "0.0",     "delay": 20.0},
    ]

    actions = []
    for r in robots:
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi_spawn_launch),
            launch_arguments={
                "robot_name": r["name"],
                "position_x": r["x"],
                "position_y": r["y"],
                "orientation_yaw": r["yaw"],
                "camera_enabled": "false",
                "stereo_camera_enabled": "false",
                "two_d_lidar_enabled": "false",
            }.items()
        )
        actions.append(TimerAction(period=r["delay"], actions=[include]))

    return LaunchDescription(actions)
