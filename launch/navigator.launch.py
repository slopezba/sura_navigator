from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")
    altitude_topic = LaunchConfiguration("altitude_topic")
    navigator_topic = LaunchConfiguration("navigator_topic")
    use_tf_fallback = LaunchConfiguration("use_tf_fallback")
    parent_frame = LaunchConfiguration("parent_frame")
    child_frame = LaunchConfiguration("child_frame")
    publish_rate = LaunchConfiguration("publish_rate")
    odom_timeout = LaunchConfiguration("odom_timeout")

    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_topic", default_value="/cirtesub/localization/odometry"),
            DeclareLaunchArgument(
                "altitude_topic",
                default_value="/cirtesub/sensors/dvl/altitude",
            ),
            DeclareLaunchArgument(
                "navigator_topic",
                default_value="/cirtesub/navigator/navigation",
            ),
            DeclareLaunchArgument("use_tf_fallback", default_value="false"),
            DeclareLaunchArgument("parent_frame", default_value="world_ned"),
            DeclareLaunchArgument("child_frame", default_value="cirtesub/base_link"),
            DeclareLaunchArgument("publish_rate", default_value="50.0"),
            DeclareLaunchArgument("odom_timeout", default_value="0.5"),
            Node(
                package="sura_navigator",
                executable="navigator_node",
                name="sura_navigator",
                output="screen",
                parameters=[
                    {
                        "odom_topic": odom_topic,
                        "altitude_topic": altitude_topic,
                        "navigator_topic": navigator_topic,
                        "use_tf_fallback": use_tf_fallback,
                        "parent_frame": parent_frame,
                        "child_frame": child_frame,
                        "publish_rate": publish_rate,
                        "odom_timeout": odom_timeout,
                    }
                ],
            ),
        ]
    )
