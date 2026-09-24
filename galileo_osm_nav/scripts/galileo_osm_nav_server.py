#!/usr/bin/env python3
"""Serve a local OSM map and a simple road-aware navigation demo."""

import argparse
import errno
import heapq
import json
import math
import mimetypes
import threading
import xml.etree.ElementTree as ET
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int8
import yaml

from pcd_overlay import PcdOverlay

# Demo profile: the robot dog emulates a pedestrian while applying conservative
# safety policies to shared carriageways. This is not a legal certification.
WALKABLE = {
    "footway", "path", "pedestrian", "steps", "living_street", "residential", "service",
    "unclassified", "tertiary", "secondary", "primary", "track"
}
DEDICATED_PEDESTRIAN_WAYS = {"footway", "path", "pedestrian", "steps"}
# Extra planning cost on shared carriageways makes dedicated pedestrian facilities preferred.
SAFETY_MULTIPLIER = {
    "footway": 1.0, "pedestrian": 1.0, "path": 1.1, "steps": 1.5,
    "living_street": 1.2, "service": 1.3, "residential": 1.5, "unclassified": 1.8,
    "track": 1.8, "tertiary": 2.5, "secondary": 3.0, "primary": 4.0,
}
DEFAULT_ROBOT_DOG_SPEED_MPS = 1.0
DEFAULT_SIGNAL_WAIT_SECONDS = 20.0
DEFAULT_MIN_ZOOM_WIDTH = 2.0
DEFAULT_SIGNAL_STOP_DISTANCE_M = 2.0
EARTH_RADIUS_M = 6_371_000


def parse_bool(value):
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError("expected a boolean: true or false")


def tags(element):
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


def is_walkable(road_type, way_tags):
    """Return whether a way is accessible to a pedestrian-like robot profile."""
    return (
        road_type in WALKABLE
        and way_tags.get("foot") not in {"no", "use_sidepath"}
        and way_tags.get("access") not in {"no", "private"}
    )


def robot_dog_oneway(road_type, way_tags):
    """Apply explicit pedestrian direction first, then vehicle direction on shared roads."""
    pedestrian_direction = way_tags.get("oneway:foot")
    if pedestrian_direction in {"yes", "-1"}:
        return pedestrian_direction
    if road_type not in DEDICATED_PEDESTRIAN_WAYS:
        vehicle_direction = way_tags.get("oneway")
        if vehicle_direction in {"yes", "-1"}:
            return vehicle_direction
    return None


def planning_cost_per_meter(road_type, robot_dog_speed_mps, safety_multipliers):
    return safety_multipliers.get(road_type, 2.0) / robot_dog_speed_mps


def distance_m(a, b):
    latitude = math.radians((a[0] + b[0]) / 2)
    dx = math.radians(a[1] - b[1]) * EARTH_RADIUS_M * math.cos(latitude)
    dy = math.radians(a[0] - b[0]) * EARTH_RADIUS_M
    return math.hypot(dx, dy)


def _pair_values(value, field_name, index):
    """Accept [first, second] or a named coordinate mapping from paired.yaml."""
    if isinstance(value, dict):
        if field_name == "world":
            keys = ("x", "y")
        else:
            keys = ("latitude", "longitude")
            if not all(key in value for key in keys):
                keys = ("lat", "lon")
        try:
            return float(value[keys[0]]), float(value[keys[1]])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"pair {index} has invalid {field_name} mapping") from error
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return float(value[0]), float(value[1])
        except (TypeError, ValueError) as error:
            raise ValueError(f"pair {index} has invalid {field_name} values") from error
    raise ValueError(f"pair {index} must contain {field_name}: [first, second]")


class SlamGpsTransform:
    """Affine least-squares transform between SLAM metres and WGS-84 coordinates."""

    def __init__(self, paired_yaml):
        paired_path = Path(paired_yaml)
        if not paired_path.is_file():
            raise ValueError(f"paired YAML does not exist: {paired_path}")
        try:
            document = yaml.safe_load(paired_path.read_text(encoding="utf-8")) or {}
        except yaml.YAMLError as error:
            raise ValueError(f"cannot parse paired YAML: {error}") from error
        pairs = document.get("pairs")
        if not isinstance(pairs, list) or len(pairs) < 3:
            raise ValueError("paired YAML needs at least 3 non-collinear entries under 'pairs'")

        slam_points, gps_points = [], []
        for index, pair in enumerate(pairs):
            if not isinstance(pair, dict):
                raise ValueError(f"pair {index} must be a mapping")
            slam_points.append(_pair_values(pair.get("world", pair.get("slam")), "world", index))
            gps_points.append(_pair_values(pair.get("gps"), "gps", index))

        self.slam_points = np.asarray(slam_points, dtype=float)
        self.gps_points = np.asarray(gps_points, dtype=float)
        self.reference_latitude = float(np.mean(self.gps_points[:, 0]))
        self.reference_longitude = float(np.mean(self.gps_points[:, 1]))
        self._cos_reference_latitude = math.cos(math.radians(self.reference_latitude))
        self.local_points = np.asarray([self._to_local(latitude, longitude) for latitude, longitude in self.gps_points])
        design = np.column_stack((np.ones(len(self.slam_points)), self.slam_points))
        if np.linalg.matrix_rank(design) < 3:
            raise ValueError("paired YAML SLAM points are collinear; use at least 3 non-collinear points")
        self.coefficients, _, _, _ = np.linalg.lstsq(design, self.local_points, rcond=None)
        self.offset = self.coefficients[0]
        self.linear = self.coefficients[1:].T
        if abs(np.linalg.det(self.linear)) < 1e-10:
            raise ValueError("paired YAML produces a singular SLAM/GPS transform")
        fitted = design @ self.coefficients
        self.rmse_m = float(np.sqrt(np.mean(np.sum((fitted - self.local_points) ** 2, axis=1))))

    def _to_local(self, latitude, longitude):
        return np.asarray((
            math.radians(longitude - self.reference_longitude) * EARTH_RADIUS_M * self._cos_reference_latitude,
            math.radians(latitude - self.reference_latitude) * EARTH_RADIUS_M,
        ))

    def _to_gps(self, east, north):
        return (
            self.reference_latitude + math.degrees(north / EARTH_RADIUS_M),
            self.reference_longitude + math.degrees(east / (EARTH_RADIUS_M * self._cos_reference_latitude)),
        )

    def slam_to_gps(self, x, y):
        east, north = self.offset + self.linear @ np.asarray((x, y))
        return self._to_gps(float(east), float(north))

    def gps_to_slam(self, latitude, longitude):
        local = self._to_local(latitude, longitude)
        x, y = np.linalg.solve(self.linear, local - self.offset)
        return float(x), float(y)

    def metadata(self):
        # Browser-side hover coordinates use the same inverse affine fit as
        # gps_to_slam(), without making a request for every mouse movement.
        inverse = np.linalg.inv(self.linear)
        metres_per_degree = EARTH_RADIUS_M * math.pi / 180.0
        local_from_lat_lon = np.asarray(((0.0, metres_per_degree * self._cos_reference_latitude),
                                         (metres_per_degree, 0.0)))
        return {
            "pair_count": int(len(self.slam_points)),
            "rmse_m": self.rmse_m,
            "gps_to_slam": {
                "reference_latitude": self.reference_latitude,
                "reference_longitude": self.reference_longitude,
                "offset": (-inverse @ self.offset).tolist(),
                "matrix": (inverse @ local_from_lat_lon).tolist(),
            },
        }


class RosNavigationBridge(Node):
    """Mode-1 ROS bridge: Odometry -> GPS and GPS route -> nav_msgs/Path."""

    def __init__(self, transform, network, odometry_topic, global_path_topic, slam_frame_id,
                 stop_nav_topic, stop_override_topic, signal_stop_distance_m):
        super().__init__("galileo_osm_nav_bridge")
        self.transform = transform
        self.network = network
        self.slam_frame_id = slam_frame_id
        self.global_path_topic = global_path_topic
        self.signal_stop_distance_m = signal_stop_distance_m
        self._position_lock = threading.Lock()
        self._latest_position = None
        self._route_points = []
        self._route_events = []
        self._route_progress = 0.0
        self._holding_signal_id = None
        self.subscription = self.create_subscription(Odometry, odometry_topic, self._odometry_callback, 10)
        self.publisher = self.create_publisher(NavPath, global_path_topic, 10)
        self.stop_publisher = self.create_publisher(Int8, stop_nav_topic, 10)
        self.zero_override_publisher = self.create_publisher(Twist, stop_override_topic, 10)
        self.create_subscription(NavPath, global_path_topic, self._path_callback, 10)
        self.create_timer(0.05, self._signal_timer)
        self.get_logger().info(
            f"Mode 1: subscribing {odometry_topic}, publishing {global_path_topic}; "
            f"signal stop={stop_nav_topic}, zero override={stop_override_topic}; "
            f"{transform.metadata()['pair_count']} pairs, RMSE {transform.rmse_m:.3f} m"
        )

    def _path_callback(self, message):
        # The downstream path bridge clears /global_path with an empty Path at goal.
        if not message.poses:
            with self._position_lock:
                self._route_points = []
                self._route_events = []
                self._holding_signal_id = None

    @staticmethod
    def _project_progress(position, points):
        best = (float("inf"), 0.0)
        travelled = 0.0
        for a, b in zip(points, points[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length_sq = dx * dx + dy * dy
            fraction = max(0.0, min(1.0, ((position[0] - a[0]) * dx + (position[1] - a[1]) * dy) / length_sq)) if length_sq else 0.0
            length = math.sqrt(length_sq)
            projection = (a[0] + fraction * dx, a[1] + fraction * dy)
            error = math.hypot(position[0] - projection[0], position[1] - projection[1])
            if error < best[0]:
                best = (error, travelled + fraction * length)
            travelled += length
        return best[1]

    def _signal_timer(self):
        with self._position_lock:
            position = self._latest_position
            points = self._route_points
            events = self._route_events
            previous_hold = self._holding_signal_id
            if position and points and events:
                self._route_progress = max(
                    self._route_progress,
                    self._project_progress((position["slam"]["x"], position["slam"]["y"]), points),
                )
            progress = self._route_progress

        holding = None
        if previous_hold and self.network.signal_state(previous_hold) == 1:
            holding = previous_hold
        elif position and points:
            for event in events:
                if self.network.signal_state(event["signal_id"]) != 1:
                    continue
                distance = event["entry_s"] - progress
                if 0.05 < distance <= self.signal_stop_distance_m:
                    holding = event["signal_id"]
                    break

        with self._position_lock:
            self._holding_signal_id = holding
        if holding:
            # /stop has other publishers that continuously send 0. The CMU mux
            # high-priority zero channel makes the red-light hold deterministic.
            self.stop_publisher.publish(Int8(data=2))
            self.zero_override_publisher.publish(Twist())
        elif previous_hold:
            self.stop_publisher.publish(Int8(data=0))
            self.get_logger().info(f"Signal {previous_hold} green: navigation resumed")
        if holding and holding != previous_hold:
            self.get_logger().info(f"Signal {holding} red: holding before crossing")

    def signal_status(self):
        with self._position_lock:
            events = [dict(event) for event in self._route_events]
            return {
                "holding_signal_id": self._holding_signal_id,
                "route_progress_m": self._route_progress,
                "route_signals": events,
                "stop_distance_m": self.signal_stop_distance_m,
            }

    def _odometry_callback(self, message):
        position = message.pose.pose.position
        latitude, longitude = self.transform.slam_to_gps(position.x, position.y)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1_000_000_000
        payload = {
            "available": True,
            "latitude": latitude,
            "longitude": longitude,
            "slam": {"x": position.x, "y": position.y, "z": position.z},
            "stamp": stamp,
        }
        with self._position_lock:
            self._latest_position = payload

    def latest_position(self):
        with self._position_lock:
            return dict(self._latest_position) if self._latest_position else {"available": False}

    def publish_global_path(self, geographic_path, crossing_events):
        message = NavPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.slam_frame_id
        slam_path = [self.transform.gps_to_slam(latitude, longitude) for latitude, longitude in geographic_path]
        cumulative = [0.0]
        for a, b in zip(slam_path, slam_path[1:]):
            cumulative.append(cumulative[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        route_events = [
            {"signal_id": event["signal_id"], "entry_s": cumulative[event["entry_index"]],
             "exit_s": cumulative[event["exit_index"]]}
            for event in crossing_events if event["entry_index"] > 0
        ]
        with self._position_lock:
            self._route_points = slam_path
            self._route_events = route_events
            self._route_progress = 0.0
            self._holding_signal_id = None
        for index, (x, y) in enumerate(slam_path):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if len(slam_path) > 1:
                next_x, next_y = slam_path[min(index + 1, len(slam_path) - 1)]
                previous_x, previous_y = slam_path[max(index - 1, 0)]
                yaw = math.atan2(next_y - previous_y, next_x - previous_x)
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
            else:
                pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.publisher.publish(message)
        return [[x, y] for x, y in slam_path]


class OSMRoadNetwork:
    def __init__(
        self, map_path, robot_dog_speed_mps=DEFAULT_ROBOT_DOG_SPEED_MPS,
        signal_wait_seconds=DEFAULT_SIGNAL_WAIT_SECONDS, safety_multipliers=None, ui_config=None
    ):
        self.map_path = Path(map_path)
        self.robot_dog_speed_mps = robot_dog_speed_mps
        self.signal_wait_seconds = signal_wait_seconds
        self.safety_multipliers = dict(SAFETY_MULTIPLIER)
        self.safety_multipliers.update(safety_multipliers or {})
        self.ui_config = ui_config or {}
        root = ET.parse(map_path).getroot()
        self.nodes = {
            node.attrib["id"]: (float(node.attrib["lat"]), float(node.attrib["lon"]))
            for node in root.findall("node")
        }
        self.traffic_signal_nodes = {
            node.attrib["id"] for node in root.findall("node")
            if tags(node).get("highway") == "traffic_signals"
            or tags(node).get("traffic_signals") == "signal"
            or tags(node).get("crossing") == "traffic_signals"
            or tags(node).get("crossing:signals") == "yes"
        }
        self.signal_coordinates = {node_id: self.nodes[node_id] for node_id in self.traffic_signal_nodes}
        self._signal_lock = threading.Lock()
        self.signal_states = {node_id: 1 for node_id in self.traffic_signal_nodes}
        self.signal_crossing_count = {node_id: 0 for node_id in self.traffic_signal_nodes}
        if not self.nodes:
            raise RuntimeError("The OSM file has no nodes.")
        lats = [point[0] for point in self.nodes.values()]
        lons = [point[1] for point in self.nodes.values()]
        self.bounds = {"min_lat": min(lats), "min_lon": min(lons), "max_lat": max(lats), "max_lon": max(lons)}
        self.reference_lat = sum(lats) / len(lats)
        self.graph, self.segments = {}, []
        self.segment_by_edge = {}
        self.roads, self.buildings, self.areas = [], [], []

        for way in root.findall("way"):
            way_tags = tags(way)
            references = [ref.attrib["ref"] for ref in way.findall("nd") if ref.attrib["ref"] in self.nodes]
            coordinates = [[self.nodes[node_id][0], self.nodes[node_id][1]] for node_id in references]
            if len(coordinates) >= 3 and way_tags.get("building"):
                self.buildings.append({"coordinates": coordinates, "name": way_tags.get("name", "")})
            area_kind = way_tags.get("landuse") or way_tags.get("natural")
            if len(coordinates) >= 3 and area_kind:
                self.areas.append({"coordinates": coordinates, "kind": area_kind, "name": way_tags.get("name", "")})

            road_type = way_tags.get("highway")
            if road_type:
                self.roads.append({
                    "coordinates": coordinates,
                    "kind": road_type,
                    "name": way_tags.get("name", ""),
                    "oneway": way_tags.get("oneway", ""),
                    "lanes": way_tags.get("lanes", ""),
                    "maxspeed": way_tags.get("maxspeed", ""),
                })
            if not is_walkable(road_type, way_tags) or len(references) < 2:
                continue

            # Dedicated pedestrian ways stay bidirectional unless explicitly marked
            # `oneway:foot`. On a shared carriageway, the demo follows OSM vehicle
            # direction to avoid routing the robot against the road flow.
            one_way = robot_dog_oneway(road_type, way_tags)
            pairs = list(zip(references, references[1:]))
            way_signals = [node_id for node_id in references if node_id in self.traffic_signal_nodes]
            # A motor-vehicle signal must not stop a pedestrian route. Only an
            # explicitly signalled pedestrian way controls the robot dog.
            synthetic_signal_id = None
            if road_type in DEDICATED_PEDESTRIAN_WAYS:
                if not way_signals and way_tags.get("crossing") == "traffic_signals":
                    synthetic_signal_id = f"way:{way.attrib['id']}"
                    self.signal_coordinates[synthetic_signal_id] = coordinates[len(coordinates) // 2]
                    self.signal_states[synthetic_signal_id] = 1
                    self.signal_crossing_count[synthetic_signal_id] = 0
            signal_controlled = bool(way_signals or synthetic_signal_id) and road_type in DEDICATED_PEDESTRIAN_WAYS
            signal_wait_per_segment = self.signal_wait_seconds / len(pairs) if signal_controlled else 0.0
            for original_start, original_end in pairs:
                signal_id = synthetic_signal_id
                if signal_controlled and way_signals:
                    midpoint = tuple((a + b) / 2 for a, b in zip(self.nodes[original_start], self.nodes[original_end]))
                    signal_id = min(way_signals, key=lambda node_id: distance_m(midpoint, self.nodes[node_id]))
                if signal_id:
                    self.signal_crossing_count[signal_id] += 1
                start, end = original_start, original_end
                if one_way == "-1":
                    start, end = end, start
                length = distance_m(self.nodes[start], self.nodes[end])
                cost_per_m = planning_cost_per_meter(
                    road_type, self.robot_dog_speed_mps, self.safety_multipliers
                ) + signal_wait_per_segment / length
                cost = length * cost_per_m
                self.graph.setdefault(start, []).append((end, cost))
                bidirectional = one_way not in {"yes", "-1"}
                if bidirectional:
                    self.graph.setdefault(end, []).append((start, cost))
                segment = {
                    "start": start,
                    "end": end,
                    "bidirectional": bidirectional,
                    "kind": road_type,
                    "name": way_tags.get("name", ""),
                    "cost_per_m": cost_per_m,
                    "signal_controlled": signal_controlled,
                    "signal_id": signal_id,
                }
                self.segments.append(segment)
                for edge in ([(start, end), (end, start)] if bidirectional else [(start, end)]):
                    if edge not in self.segment_by_edge or signal_controlled:
                        self.segment_by_edge[edge] = segment

        # Map data is immutable while the server is running. Build and encode both
        # display variants once at startup so a large OSM file is not repeatedly
        # traversed and JSON-serialized for every browser refresh/layer switch.
        self._map_payloads = {
            "compact": self._build_map_data("compact"),
            "full": self._build_map_data("full"),
        }
        self._map_json = {
            detail: json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            for detail, payload in self._map_payloads.items()
        }

    def map_data(self, detail="compact"):
        """Return a light road-only map by default; full mode is for inspection."""
        if detail not in {"compact", "full"}:
            raise ValueError("map detail must be compact or full")
        return self._map_payloads[detail]

    def map_json(self, detail="compact"):
        """Return the pre-encoded immutable map response for a display mode."""
        if detail not in {"compact", "full"}:
            raise ValueError("map detail must be compact or full")
        return self._map_json[detail]

    def _build_map_data(self, detail):
        if detail == "compact":
            return {
                "bounds": self.bounds,
                "detail": "compact",
                # Deliberately omit labels, nodes, buildings and areas. This keeps
                # large raw OSM maps responsive in the browser.
                "roads": [
                    {"coordinates": road["coordinates"], "kind": road["kind"], "oneway": road["oneway"]}
                    for road in self.roads
                    if len(road["coordinates"]) >= 2
                ],
            }
        return {
            "bounds": self.bounds,
            "detail": "full",
            "roads": self.roads,
            "buildings": self.buildings,
            "areas": self.areas,
            "nodes": [[node_id, point[0], point[1]] for node_id, point in self.nodes.items()],
            "traffic_signals": [
                [node_id, self.nodes[node_id][0], self.nodes[node_id][1]]
                for node_id in sorted(self.traffic_signal_nodes)
            ],
        }

    def config_data(self):
        return {
            "map_name": self.map_path.name,
            "default_display_mode": self.ui_config.get("default_display_mode", "compact"),
            "min_zoom_width": self.ui_config.get("min_zoom_width", DEFAULT_MIN_ZOOM_WIDTH),
            "use_leaflet": self.ui_config.get("use_leaflet", True),
            "mode": self.ui_config.get("mode", 0),
            "global_path_topic": self.ui_config.get("global_path_topic", "/global_path"),
            "transform": self.ui_config.get("transform"),
            "signal_stop_distance_m": self.ui_config.get("signal_stop_distance_m", DEFAULT_SIGNAL_STOP_DISTANCE_M),
        }

    def signal_state(self, signal_id):
        with self._signal_lock:
            return self.signal_states[signal_id]

    def set_signal_state(self, signal_id, state):
        if type(state) is not int or state not in (0, 1):
            raise ValueError("signal state must be 0 (green) or 1 (red)")
        with self._signal_lock:
            if signal_id not in self.signal_states:
                raise KeyError(signal_id)
            self.signal_states[signal_id] = state

    def signals_data(self):
        with self._signal_lock:
            return [{"id": signal_id, "latitude": point[0], "longitude": point[1],
                     "state": self.signal_states[signal_id],
                     "controls_pedestrian_crossing": self.signal_crossing_count[signal_id] > 0}
                    for signal_id, point in sorted(self.signal_coordinates.items())]

    def _to_local(self, point):
        return (
            math.radians(point[1]) * EARTH_RADIUS_M * math.cos(math.radians(self.reference_lat)),
            math.radians(point[0]) * EARTH_RADIUS_M,
        )

    def _project_to_segment(self, point, start, end):
        px, py = self._to_local(point)
        ax, ay = self._to_local(start)
        bx, by = self._to_local(end)
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        fraction = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        projected = (start[0] + (end[0] - start[0]) * fraction, start[1] + (end[1] - start[1]) * fraction)
        return fraction, projected, distance_m(point, projected)

    def snap_to_road(self, point):
        if not self.segments:
            raise ValueError("The OSM map has no walkable road segments for the robot-dog profile.")
        best = None
        for segment in self.segments:
            fraction, projected, snap_distance = self._project_to_segment(
                point, self.nodes[segment["start"]], self.nodes[segment["end"]]
            )
            candidate = (snap_distance, fraction, projected, segment)
            if best is None or candidate[0] < best[0]:
                best = candidate
        snap_distance, fraction, projected, segment = best
        return {
            "coordinate": projected,
            "distance_m": snap_distance,
            "fraction": fraction,
            "segment": segment,
            "road": {"name": segment["name"], "kind": segment["kind"]},
        }

    def _graph_with_virtual_points(self, start_snap, goal_snap):
        graph = {node_id: list(edges) for node_id, edges in self.graph.items()}
        coordinates = dict(self.nodes)

        def attach(key, snap):
            segment = snap["segment"]
            start, end = segment["start"], segment["end"]
            coordinate = snap["coordinate"]
            coordinates[key] = coordinate
            graph.setdefault(key, [])
            first_cost = distance_m(self.nodes[start], coordinate) * segment["cost_per_m"]
            second_cost = distance_m(coordinate, self.nodes[end]) * segment["cost_per_m"]
            # Keeping the original direct edge is intentional: it permits independent
            # start/goal projections on the same segment without modifying the base graph.
            graph.setdefault(start, []).append((key, first_cost))
            graph[key].append((end, second_cost))
            if segment["bidirectional"]:
                graph.setdefault(end, []).append((key, second_cost))
                graph[key].append((start, first_cost))

        attach("__start__", start_snap)
        attach("__goal__", goal_snap)
        return graph, coordinates

    def route(self, start_point, goal_point):
        start_snap = self.snap_to_road(start_point)
        goal_snap = self.snap_to_road(goal_point)
        graph, coordinates = self._graph_with_virtual_points(start_snap, goal_snap)
        start, goal = "__start__", "__goal__"
        costs, previous = {start: 0.0}, {}
        queue = [(0.0, start)]
        while queue:
            cost, current = heapq.heappop(queue)
            if cost != costs[current]:
                continue
            if current == goal:
                break
            for neighbor, edge_cost in graph.get(current, []):
                candidate = cost + edge_cost
                if candidate < costs.get(neighbor, float("inf")):
                    costs[neighbor] = candidate
                    previous[neighbor] = current
                    heapq.heappush(queue, (candidate, neighbor))
        if goal not in costs:
            raise ValueError("The selected start and goal are not connected in the walkable robot-dog topology.")
        path = [goal]
        while path[-1] != start:
            path.append(previous[path[-1]])
        path.reverse()
        path_coordinates = [[coordinates[node_id][0], coordinates[node_id][1]] for node_id in path]
        physical_distance = sum(distance_m(a, b) for a, b in zip(path_coordinates, path_coordinates[1:]))
        crossing_events = []
        for index, (a, b) in enumerate(zip(path, path[1:])):
            if a == "__start__":
                segment = start_snap["segment"]
            elif b == "__goal__":
                segment = goal_snap["segment"]
            else:
                segment = self.segment_by_edge.get((a, b))
            signal_id = segment["signal_id"] if segment else None
            if signal_id:
                if crossing_events and crossing_events[-1]["signal_id"] == signal_id and crossing_events[-1]["exit_index"] == index:
                    crossing_events[-1]["exit_index"] = index + 1
                else:
                    crossing_events.append({"signal_id": signal_id, "entry_index": index, "exit_index": index + 1})
        signal_count = len(crossing_events)
        return {
            "path": path_coordinates,
            "distance_m": physical_distance,
            "planning_cost_s": costs[goal],
            "traffic_signal_count": signal_count,
            "signal_wait_seconds": signal_count * self.signal_wait_seconds,
            "crossing_events": crossing_events,
            "start": start_snap,
            "goal": goal_snap,
        }


def default_map_path():
    return Path(get_package_share_directory("lanelet2_maps")) / "res" / "map_galileo.osm"


def make_handler(network, web_dir, mode=0, ros_bridge=None, pcd_overlay=None):
    web_dir = web_dir.resolve()

    class Handler(BaseHTTPRequestHandler):
        def send_bytes(self, content, content_type, status=HTTPStatus.OK):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def send_json(self, payload, status=HTTPStatus.OK):
            content = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_bytes(content, "application/json; charset=utf-8", status)

        def do_GET(self):
            request = urlparse(self.path)
            if request.path == "/api/config":
                self.send_json(network.config_data())
                return
            if request.path == "/api/map":
                try:
                    detail = parse_qs(request.query).get("detail", ["compact"])[0]
                    self.send_bytes(network.map_json(detail), "application/json; charset=utf-8")
                except ValueError as error:
                    self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
                return
            if request.path == "/api/buildings":
                self.send_json({"buildings": network.buildings})
                return
            if request.path == "/api/pcd":
                self.send_json(pcd_overlay.metadata() if pcd_overlay else {"available": False})
                return
            if request.path == "/api/pcd/overlay":
                if pcd_overlay is None:
                    self.send_json({"error": "PCD overlay is disabled"}, HTTPStatus.NOT_FOUND)
                    return
                query = parse_qs(request.query)
                try:
                    content = pcd_overlay.png(query.get("frame", ["raw"])[0], query.get("height", ["high"])[0])
                    self.send_bytes(content, "image/png")
                except ValueError as error:
                    self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
                return
            if request.path == "/api/robot_pose":
                if mode != 1 or ros_bridge is None:
                    self.send_json({"available": False, "mode": mode})
                else:
                    self.send_json(ros_bridge.latest_position())
                return
            if request.path == "/api/signals":
                self.send_json({"signals": network.signals_data(),
                                "navigation": ros_bridge.signal_status() if ros_bridge else None})
                return
            if request.path == "/api/route":
                try:
                    query = parse_qs(request.query)
                    goal = (float(query["goal_lat"][0]), float(query["goal_lon"][0]))
                    if mode == 1:
                        if ros_bridge is None:
                            raise ValueError("mode 1 ROS bridge is unavailable")
                        latest = ros_bridge.latest_position()
                        if not latest.get("available"):
                            raise ValueError("waiting for a message on the configured Odometry topic")
                        start = (latest["latitude"], latest["longitude"])
                    else:
                        start = (float(query["start_lat"][0]), float(query["start_lon"][0]))
                    route = network.route(start, goal)
                    if mode == 1:
                        route["slam_path"] = ros_bridge.publish_global_path(route["path"], route["crossing_events"])
                        route["global_path_topic"] = ros_bridge.global_path_topic
                        route["start_source"] = "odometry"
                    self.send_json(route)
                except (KeyError, ValueError) as error:
                    self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
                return

            relative = Path("index.html" if request.path == "/" else request.path.lstrip("/"))
            if relative.is_absolute() or ".." in relative.parts:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            candidate = web_dir / relative
            if not candidate.is_file():
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            content = candidate.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", mimetypes.guess_type(candidate.name)[0] or "application/octet-stream")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

        def do_POST(self):
            request = urlparse(self.path)
            if not request.path.startswith("/api/signals/"):
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            signal_id = unquote(request.path.removeprefix("/api/signals/"))
            try:
                content_length = int(self.headers.get("Content-Length", "0"))
                if not 0 < content_length <= 1024:
                    raise ValueError("signal request body must be between 1 and 1024 bytes")
                payload = json.loads(self.rfile.read(content_length))
                if not isinstance(payload, dict):
                    raise ValueError("signal request must be a JSON object")
                network.set_signal_state(signal_id, payload.get("state"))
                self.send_json({"signals": network.signals_data(),
                                "navigation": ros_bridge.signal_status() if ros_bridge else None})
            except KeyError:
                self.send_json({"error": f"unknown signal id: {signal_id}"}, HTTPStatus.NOT_FOUND)
            except (ValueError, json.JSONDecodeError) as error:
                self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)

        def log_message(self, format_string, *args):
            print(f"[web] {format_string % args}")

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=default_map_path(), help="Standard OSM map file")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--robot-dog-speed-mps", type=float, default=DEFAULT_ROBOT_DOG_SPEED_MPS)
    parser.add_argument("--signal-wait-seconds", type=float, default=DEFAULT_SIGNAL_WAIT_SECONDS)
    parser.add_argument("--safety-multipliers", default=json.dumps(SAFETY_MULTIPLIER))
    parser.add_argument("--default-display-mode", choices=("compact", "full"), default="compact")
    parser.add_argument("--min-zoom-width", type=float, default=DEFAULT_MIN_ZOOM_WIDTH)
    parser.add_argument("--use-leaflet", type=parse_bool, default=True)
    parser.add_argument("--mode", type=int, choices=(0, 1), default=0)
    parser.add_argument("--odometry-topic", default="/Odometry")
    parser.add_argument("--global-path-topic", default="/global_path")
    parser.add_argument("--slam-frame-id", default="map")
    parser.add_argument("--paired-yaml", type=Path)
    parser.add_argument("--stop-nav", default="/stop")
    parser.add_argument("--stop-override-topic", default="/cmd_vel/final_align")
    parser.add_argument("--signal-stop-distance-m", type=float, default=DEFAULT_SIGNAL_STOP_DISTANCE_M)
    parser.add_argument("--pcd-overlay", type=parse_bool, default=False)
    parser.add_argument("--pcd-map-dir", type=Path)
    args = parser.parse_args()
    if not args.map.is_file():
        parser.error(f"map file does not exist: {args.map}")
    if args.robot_dog_speed_mps <= 0 or args.signal_wait_seconds < 0 or args.min_zoom_width <= 0 or args.signal_stop_distance_m <= 0:
        parser.error("speed, minimum zoom width and signal stop distance must be positive; signal wait cannot be negative")
    server = None
    try:
        safety_multipliers = json.loads(args.safety_multipliers)
        if not isinstance(safety_multipliers, dict):
            raise ValueError("must be a JSON object")
        safety_multipliers = {str(key): float(value) for key, value in safety_multipliers.items()}
    except (ValueError, TypeError, json.JSONDecodeError) as error:
        parser.error(f"invalid --safety-multipliers: {error}")

    network = OSMRoadNetwork(
        args.map, args.robot_dog_speed_mps, args.signal_wait_seconds, safety_multipliers,
        {
            "default_display_mode": args.default_display_mode,
            "min_zoom_width": args.min_zoom_width,
            "use_leaflet": args.use_leaflet,
            "mode": args.mode,
            "global_path_topic": args.global_path_topic,
            "signal_stop_distance_m": args.signal_stop_distance_m,
        },
    )
    pcd_overlay = None
    if args.pcd_overlay:
        if args.pcd_map_dir is None:
            parser.error("--pcd-map-dir is required when --pcd-overlay is true")
        try:
            pcd_overlay = PcdOverlay(args.pcd_map_dir)
        except (OSError, KeyError, ValueError) as error:
            parser.error(f"cannot load PCD overlay: {error}")
    ros_bridge = None
    ros_executor = None
    ros_thread = None
    transform_metadata = None
    if args.mode == 1:
        if args.paired_yaml is None:
            parser.error("--paired-yaml is required when --mode 1")
        try:
            transform = SlamGpsTransform(args.paired_yaml)
        except ValueError as error:
            parser.error(f"invalid paired transform: {error}")
        transform_metadata = transform.metadata()
        rclpy.init(args=None)
        ros_bridge = RosNavigationBridge(
            transform, network, args.odometry_topic, args.global_path_topic, args.slam_frame_id,
            args.stop_nav, args.stop_override_topic, args.signal_stop_distance_m,
        )
        ros_executor = MultiThreadedExecutor()
        ros_executor.add_node(ros_bridge)
        ros_thread = threading.Thread(target=ros_executor.spin, name="galileo_ros_executor", daemon=True)
        ros_thread.start()

    package_share = Path(get_package_share_directory("galileo_osm_nav"))
    network.ui_config["transform"] = transform_metadata
    try:
        server = ThreadingHTTPServer(
            (args.host, args.port), make_handler(network, package_share / "web", args.mode, ros_bridge, pcd_overlay)
        )
    except OSError as error:
        if error.errno == errno.EADDRINUSE:
            parser.error(f"port {args.port} is already in use; close the existing server or choose --port <number>")
        raise
    print(f"Map: {args.map}")
    print(f"Open http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            server.server_close()
        if ros_executor is not None:
            ros_executor.shutdown()
        if ros_bridge is not None:
            ros_bridge.destroy_node()
        if args.mode == 1 and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
