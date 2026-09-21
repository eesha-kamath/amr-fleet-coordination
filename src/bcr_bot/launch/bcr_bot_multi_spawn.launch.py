#!/usr/bin/python3

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    bcr_bot_path = get_package_share_directory("bcr_bot")

    robot_name = LaunchConfiguration("robot_name").perform(context)
    position_x = LaunchConfiguration("position_x").perform(context)
    position_y = LaunchConfiguration("position_y").perform(context)
    orientation_yaw = LaunchConfiguration("orientation_yaw").perform(context)
    camera_enabled = LaunchConfiguration("camera_enabled").perform(context)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled").perform(context)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled").perform(context)
    odometry_source = LaunchConfiguration("odometry_source").perform(context)

    wheel_odom_topic = f"{robot_name}/odom"
    robot_description_topic = f"/{robot_name}/robot_description"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"robot_state_publisher_{robot_name}",
        parameters=[{
            'robot_description': Command([
                'xacro ', join(bcr_bot_path, 'urdf/bcr_bot.xacro'),
                ' camera_enabled:=', camera_enabled,
                ' stereo_camera_enabled:=', stereo_camera_enabled,
                ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                ' odometry_source:=', odometry_source,
                ' robot_name:=', robot_name,
                ' wheel_odom_topic:=', wheel_odom_topic,
                ' sim_gz:=', "true"
            ])
        }],
        remappings=[
            ('/joint_states', f'/{robot_name}/joint_states'),
            ('/robot_description', robot_description_topic),
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", robot_description_topic,
            "-name", robot_name,
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"ros_gz_bridge_{robot_name}",
        arguments=[
            f"{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"{wheel_odom_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{robot_name}/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{robot_name}/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{robot_name}/stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ]
    )

    gz_ros2_bridge_extra = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"ros_gz_bridge_extra_{robot_name}",
        arguments=[
            f"{robot_name}/kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{robot_name}/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{robot_name}/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{robot_name}/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            f"{robot_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            f"/world/default/model/{robot_name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        remappings=[
            (f'/world/default/model/{robot_name}/joint_state', f'/{robot_name}/joint_states'),
        ]
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"kinect_tf_{robot_name}",
        arguments=["--x", "0.0",
                   "--y", "0.0",
                   "--z", "0.0",
                   "--yaw", "0.0",
                   "--pitch", "0.0",
                   "--roll", "0.0",
                   "--frame-id", f"{robot_name}/kinect_camera",
                   "--child-frame-id", f"{robot_name}/base_footprint/kinect_camera"]
    )

    return [robot_state_publisher, gz_spawn_entity, transform_publisher, gz_ros2_bridge, gz_ros2_bridge_extra]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="bcr_bot"),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("camera_enabled", default_value="true"),
        DeclareLaunchArgument("stereo_camera_enabled", default_value="false"),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value="true"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        OpaqueFunction(function=launch_setup)
    ])
