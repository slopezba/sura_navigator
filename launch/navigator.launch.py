from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    environment = LaunchConfiguration("environment").perform(context)
    localization = LaunchConfiguration("localization").perform(context)

    if environment not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported environment '{environment}'. Use 'sim' or 'real'."
        )

    if localization not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported localization '{localization}'. Use 'sim' or 'real'."
        )

    def topic(path):
        if robot_namespace:
            return f"/{robot_namespace}/{path}"
        return f"/{path}"

    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    if not odom_topic:
        odom_topic = topic("localization/odometry")

    altitude_topic = LaunchConfiguration("altitude_topic").perform(context)
    if not altitude_topic:
        altitude_topic = topic("sensors/dvl/altitude")

    navigator_topic = LaunchConfiguration("navigator_topic").perform(context)
    if not navigator_topic:
        navigator_topic = topic("navigator/navigation")

    legacy_navigator_topic = LaunchConfiguration("legacy_navigator_topic").perform(context)
    if not legacy_navigator_topic:
        legacy_navigator_topic = topic("catamaran/navigator_msg")

    child_frame = LaunchConfiguration("child_frame").perform(context)
    if not child_frame:
        child_frame = f"{robot_namespace}/base_link" if robot_namespace else "base_link"

    twist_odom_topic = LaunchConfiguration("twist_odom_topic").perform(context)
    if not twist_odom_topic and environment == "sim":
        twist_odom_topic = topic("catamaran/odometry")

    use_sim_localization = environment == "sim" and localization == "sim"
    executable = "navigator_sim" if use_sim_localization else "navigator_node"
    node_name = "navigator_sim" if use_sim_localization else "sura_navigator"

    parameters = {
        "altitude_topic": altitude_topic,
        "navigator_topic": navigator_topic,
        "legacy_navigator_topic": legacy_navigator_topic,
        "parent_frame": LaunchConfiguration("parent_frame"),
        "child_frame": child_frame,
        "publish_rate": LaunchConfiguration("publish_rate"),
    }

    if not use_sim_localization:
        parameters.update(
            {
                "odom_topic": odom_topic,
                "twist_odom_topic": twist_odom_topic,
                "odom_twist_in_body_frame": LaunchConfiguration("odom_twist_in_body_frame"),
                "odom_invert_angular_z": LaunchConfiguration("odom_invert_angular_z"),
                "twist_odom_twist_in_body_frame": LaunchConfiguration("twist_odom_twist_in_body_frame"),
                "twist_odom_invert_angular_z": LaunchConfiguration("twist_odom_invert_angular_z"),
                "linear_lpf_alpha": LaunchConfiguration("linear_lpf_alpha"),
                "use_tf_fallback": LaunchConfiguration("use_tf_fallback"),
                "odom_timeout": LaunchConfiguration("odom_timeout"),
            }
        )

    adapter_parameters = {
        "input_topic": LaunchConfiguration("setpoint_input_topic").perform(context)
        or topic("catamaran/body_position/setpoint_world"),
        "output_topic": LaunchConfiguration("setpoint_output_topic").perform(context)
        or topic("catamaran/body_position/setpoint"),
        "target_frame": LaunchConfiguration("setpoint_target_frame"),
        "transform_timeout": LaunchConfiguration("setpoint_transform_timeout"),
    }

    return [
        Node(
            package="sura_navigator",
            executable=executable,
            name=node_name,
            namespace=robot_namespace,
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="sura_navigator",
            executable="position_setpoint_adapter",
            name="position_setpoint_adapter",
            namespace=robot_namespace,
            output="screen",
            parameters=[adapter_parameters],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument("environment", default_value="sim"),
            DeclareLaunchArgument("localization", default_value="real"),
            DeclareLaunchArgument("odom_topic", default_value=""),
            DeclareLaunchArgument("twist_odom_topic", default_value=""),
            DeclareLaunchArgument("altitude_topic", default_value=""),
            DeclareLaunchArgument("navigator_topic", default_value=""),
            DeclareLaunchArgument("legacy_navigator_topic", default_value=""),
            DeclareLaunchArgument("odom_twist_in_body_frame", default_value="false"),
            DeclareLaunchArgument("odom_invert_angular_z", default_value="true"),
            DeclareLaunchArgument("twist_odom_twist_in_body_frame", default_value="true"),
            DeclareLaunchArgument("twist_odom_invert_angular_z", default_value="false"),
            DeclareLaunchArgument("linear_lpf_alpha", default_value="0.2"),
            DeclareLaunchArgument("use_tf_fallback", default_value="false"),
            DeclareLaunchArgument("parent_frame", default_value="world_ned"),
            DeclareLaunchArgument("child_frame", default_value=""),
            DeclareLaunchArgument("publish_rate", default_value="50.0"),
            DeclareLaunchArgument("odom_timeout", default_value="0.5"),
            DeclareLaunchArgument("setpoint_input_topic", default_value=""),
            DeclareLaunchArgument("setpoint_output_topic", default_value=""),
            DeclareLaunchArgument("setpoint_target_frame", default_value="map"),
            DeclareLaunchArgument("setpoint_transform_timeout", default_value="0.2"),
            OpaqueFunction(function=launch_setup),
        ]
    )
