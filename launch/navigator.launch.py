from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bool_launch_arg(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False

    raise RuntimeError(
        f"Unsupported value '{value}' for launch argument '{name}'. Use true or false."
    )


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    if not robot_namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")

    environment = LaunchConfiguration("environment").perform(context)
    localization = LaunchConfiguration("localization").perform(context)
    publish_tf = bool_launch_arg(context, "publish_tf")

    if environment not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported environment '{environment}'. Use 'sim' or 'real'."
        )
    if localization not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported localization '{localization}'. Use 'sim' or 'real'."
        )

    odom_topic = (
        f"/{robot_namespace}/stonefish/odometry"
        if localization == "sim"
        else f"/{robot_namespace}/odometry/filtered"
    )

    parameters = {
        "odom_topic": odom_topic,
        "altitude_topic": f"/{robot_namespace}/sensors/altitude",
        "navigator_topic": f"/{robot_namespace}/navigator/navigation",
        "parent_frame": "world_ned",
        "child_frame": f"{robot_namespace}/base_link",
        "publish_tf": publish_tf,
    }

    adapter_parameters = {
        "input_topic": LaunchConfiguration("setpoint_input_topic").perform(context)
        or "body_position/setpoint_world",
        "output_topic": LaunchConfiguration("setpoint_output_topic").perform(context)
        or "controller/body_position/setpoint",
        "target_frame": LaunchConfiguration("setpoint_target_frame"),
        "transform_timeout": LaunchConfiguration("setpoint_transform_timeout"),
    }
    if not adapter_parameters["target_frame"].perform(context):
        adapter_parameters["target_frame"] = f"{robot_namespace}/map"

    nodes = [
        Node(
            package="sura_navigator",
            executable="navigator",
            name="sura_navigator",
            output="screen",
            parameters=[parameters],
        ),
    ]

    return [
        GroupAction(nodes),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace"),
            DeclareLaunchArgument("environment", default_value="sim"),
            DeclareLaunchArgument("localization", default_value="real"),
            DeclareLaunchArgument("localization_frame_convention", default_value="ned"),
            DeclareLaunchArgument("odom_topic", default_value=""),
            DeclareLaunchArgument("twist_odom_topic", default_value=""),
            DeclareLaunchArgument("altitude_topic", default_value=""),
            DeclareLaunchArgument("navigator_topic", default_value=""),
            DeclareLaunchArgument("legacy_navigator_topic", default_value=""),
            DeclareLaunchArgument("odom_twist_in_body_frame", default_value="true"),
            DeclareLaunchArgument("odom_invert_angular_z", default_value="true"),
            DeclareLaunchArgument("twist_odom_twist_in_body_frame", default_value="true"),
            DeclareLaunchArgument("twist_odom_invert_angular_z", default_value="false"),
            DeclareLaunchArgument("linear_lpf_alpha", default_value="0.2"),
            DeclareLaunchArgument("use_tf_fallback", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("setpoint_input_topic", default_value=""),
            DeclareLaunchArgument("setpoint_output_topic", default_value=""),
            DeclareLaunchArgument("setpoint_target_frame", default_value=""),
            DeclareLaunchArgument("setpoint_transform_timeout", default_value="0.2"),
            OpaqueFunction(function=launch_setup),
        ]
    )
