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
    package_share = Path(get_package_share_directory("galileo_osm_nav"))
    config_path = package_share / "config" / "galileo_osm_nav.yaml"
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    return data["galileo_osm_nav"]["ros__parameters"]


def resolve_map_path(map_file):
    candidate = Path(str(map_file))
    if candidate.is_absolute():
        return str(candidate)
    return str(Path(get_package_share_directory("lanelet2_maps")) / "res" / candidate)


def generate_launch_description():
    parameters = load_parameters()
    default_map = resolve_map_path(parameters["map_file"])
    executable = str(
        Path(get_package_prefix("galileo_osm_nav"))
        / "lib" / "galileo_osm_nav" / "galileo_osm_nav_server.py"
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
        DeclareLaunchArgument("use_leaflet", default_value=str(parameters["use_leaflet"]).lower()),
        DeclareLaunchArgument("pcd_overlay", default_value=str(parameters.get("pcd_overlay", False)).lower()),
        DeclareLaunchArgument("pcd_map_dir", default_value=str(parameters.get("pcd_map_dir", ""))),
        DeclareLaunchArgument("mode", default_value=str(parameters["mode"])),
        DeclareLaunchArgument("odometry_topic", default_value=str(parameters["odometry_topic"])),
        DeclareLaunchArgument("global_path_topic", default_value=str(parameters["global_path_topic"])),
        DeclareLaunchArgument("slam_frame_id", default_value=str(parameters["slam_frame_id"])),
        DeclareLaunchArgument("paired_yaml", default_value=str(parameters["paired_yaml"])),
        DeclareLaunchArgument("stop_nav", default_value=str(parameters.get("stop_nav", "/stop"))),
        DeclareLaunchArgument("stop_override_topic", default_value=str(parameters.get("stop_override_topic", "/cmd_vel/final_align"))),
        DeclareLaunchArgument("signal_stop_distance_m", default_value=str(parameters.get("signal_stop_distance_m", 2.0))),
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
                "--use-leaflet", LaunchConfiguration("use_leaflet"),
                "--pcd-overlay", LaunchConfiguration("pcd_overlay"),
                "--pcd-map-dir", LaunchConfiguration("pcd_map_dir"),
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
