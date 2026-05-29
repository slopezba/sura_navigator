from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    if not robot_namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")

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

    localization_frame_convention = LaunchConfiguration(
        "localization_frame_convention"
    ).perform(context)
    if localization_frame_convention not in ("ned", "enu"):
        raise RuntimeError(
            "Launch argument 'localization_frame_convention' must be 'ned' or 'enu'."
        )

    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    if not odom_topic:
        odom_topic = (
            "odometry/filtered"
            if localization_frame_convention == "ned"
            else "localization/odometry"
        )

    altitude_topic = LaunchConfiguration("altitude_topic").perform(context)
    if not altitude_topic:
        altitude_topic = "sensors/dvl/altitude"

    navigator_topic = LaunchConfiguration("navigator_topic").perform(context)
    if not navigator_topic:
        navigator_topic = "navigator/navigation"

    legacy_navigator_topic = LaunchConfiguration("legacy_navigator_topic").perform(context)
    if not legacy_navigator_topic:
        legacy_navigator_topic = "navigator/msg"

    child_frame = LaunchConfiguration("child_frame").perform(context)
    if not child_frame:
        child_frame = f"{robot_namespace}/base_link"

    twist_odom_topic = LaunchConfiguration("twist_odom_topic").perform(context)
    if not twist_odom_topic and environment == "sim":
        twist_odom_topic = "odometry"

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
            executable=executable,
            name=node_name,
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="sura_navigator",
            executable="position_setpoint_adapter",
            name="position_setpoint_adapter",
            output="screen",
            parameters=[adapter_parameters],
        ),
    ]

    if use_sim_localization:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("cirtesu_tank_aruco_localization"),
                            "launch",
                            "aruco_map_localization.launch.py",
                        ]
                    )
                ),
                launch_arguments=[
                    ("robot_namespace", robot_namespace),
                ],
            )
        )

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
            DeclareLaunchArgument("setpoint_target_frame", default_value=""),
            DeclareLaunchArgument("setpoint_transform_timeout", default_value="0.2"),
            OpaqueFunction(function=launch_setup),
        ]
    )
