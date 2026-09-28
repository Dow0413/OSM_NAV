#!/usr/bin/env python3
"""Launch Galileo OSM navigation using the package YAML configuration."""

import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def load_parameters():
    package_share = Path(get_package_share_directory("osm_nav"))
    config_path = package_share / "config" / "osm_nav.yaml"
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    return data["osm_nav"]["ros__parameters"]


def resolve_map_path(map_file):
    candidate = Path(str(map_file))
    if candidate.is_absolute():
        return str(candidate)
    return str(Path(get_package_share_directory("osm_nav")) / "maps" / candidate)


def resolve_config_path(path):
    candidate = Path(str(path))
    if candidate.is_absolute():
        return str(candidate)
    return str(Path(get_package_share_directory("osm_nav")) / "config" / candidate)


def mode_parameters(parameters):
    """Return the configuration section matching the configured mode.

    Mode-specific settings deliberately stay out of the top-level ROS
    parameters. This prevents a growing list of unrelated mode options from
    being presented as one flat configuration block.
    """
    try:
        mode = int(parameters["mode"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("osm_nav.mode must be an integer") from error
    section_name = f"mode_{mode}"
    section = parameters.get(section_name)
    if not isinstance(section, dict):
        raise ValueError(
            f"osm_nav requires a mapping named '{section_name}' for mode {mode}"
        )
    return section


def generate_launch_description():
    parameters = load_parameters()
    selected_mode_parameters = mode_parameters(parameters)
    default_map = resolve_map_path(parameters["map_file"])
    executable = str(
        Path(get_package_prefix("osm_nav"))
        / "lib" / "osm_nav" / "osm_nav_server.py"
    )
    safety_multipliers = json.dumps(parameters["safety_multipliers"])

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map, description="OSM/XML map path"),
        DeclareLaunchArgument("host", default_value=str(parameters["host"])),
        DeclareLaunchArgument("port", default_value=str(parameters["port"])),
        DeclareLaunchArgument("robot_dog_speed_mps", default_value=str(parameters["robot_dog_speed_mps"])),
        DeclareLaunchArgument("signal_wait_seconds", default_value=str(parameters["signal_wait_seconds"])),
        DeclareLaunchArgument("default_display_mode", default_value=str(parameters["default_display_mode"])),
        DeclareLaunchArgument("min_zoom_width", default_value=str(parameters["min_zoom_width"])),
        DeclareLaunchArgument("use_base_map", default_value=str(parameters.get("use_base_map", "esri_satellite"))),
        DeclareLaunchArgument("pcd_overlay", default_value=str(parameters.get("pcd_overlay", False)).lower()),
        DeclareLaunchArgument("pcd_dir", default_value=str(parameters.get("pcd_dir", ""))),
        DeclareLaunchArgument("manifest_yaml_dir", default_value=str(parameters.get("manifest_yaml_dir", ""))),
        DeclareLaunchArgument("mode", default_value=str(parameters["mode"])),
        # These defaults come from mode_<mode>, rather than the common block.
        # They remain launch arguments so a one-off CLI override still works.
        DeclareLaunchArgument("odometry_topic", default_value=str(selected_mode_parameters.get("odometry_topic", "/Odometry"))),
        DeclareLaunchArgument("global_path_topic", default_value=str(selected_mode_parameters.get("global_path_topic", "/global_path"))),
        DeclareLaunchArgument("slam_frame_id", default_value=str(selected_mode_parameters.get("slam_frame_id", "map"))),
        DeclareLaunchArgument("paired_yaml", default_value=resolve_config_path(selected_mode_parameters.get("paired_yaml", ""))),
        DeclareLaunchArgument("stop_nav", default_value=str(selected_mode_parameters.get("stop_nav", "/stop"))),
        DeclareLaunchArgument("stop_override_topic", default_value=str(selected_mode_parameters.get("stop_override_topic", "/cmd_vel/final_align"))),
        DeclareLaunchArgument("signal_stop_distance_m", default_value=str(selected_mode_parameters.get("signal_stop_distance_m", 2.0))),
        ExecuteProcess(
            cmd=[
                executable,
                "--map", LaunchConfiguration("map"),
                "--host", LaunchConfiguration("host"),
                "--port", LaunchConfiguration("port"),
                "--robot-dog-speed-mps", LaunchConfiguration("robot_dog_speed_mps"),
                "--signal-wait-seconds", LaunchConfiguration("signal_wait_seconds"),
                "--safety-multipliers", safety_multipliers,
                "--default-display-mode", LaunchConfiguration("default_display_mode"),
                "--min-zoom-width", LaunchConfiguration("min_zoom_width"),
                "--use-base-map", LaunchConfiguration("use_base_map"),
                "--pcd-overlay", LaunchConfiguration("pcd_overlay"),
                "--pcd-dir", LaunchConfiguration("pcd_dir"),
                "--manifest-yaml-dir", LaunchConfiguration("manifest_yaml_dir"),
                "--mode", LaunchConfiguration("mode"),
                "--odometry-topic", LaunchConfiguration("odometry_topic"),
                "--global-path-topic", LaunchConfiguration("global_path_topic"),
                "--slam-frame-id", LaunchConfiguration("slam_frame_id"),
                "--paired-yaml", LaunchConfiguration("paired_yaml"),
                "--stop-nav", LaunchConfiguration("stop_nav"),
                "--stop-override-topic", LaunchConfiguration("stop_override_topic"),
                "--signal-stop-distance-m", LaunchConfiguration("signal_stop_distance_m"),
            ],
            output="screen",
        ),
    ])
