#!/usr/bin/env python3
"""Serve a local OSM map and a simple road-aware navigation demo."""

import argparse
import errno
import json
import math
import shutil
import mimetypes
import sys
import threading
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
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

# Keep algorithm modules discoverable in both layouts: ``scripts/../src`` in
# a checkout and ``lib/osm_nav/src`` after ament installs the executable.
SCRIPT_DIR = Path(__file__).resolve().parent
for module_root in (SCRIPT_DIR / "src", SCRIPT_DIR.parent / "src"):
    if module_root.is_dir() and str(module_root) not in sys.path:
        sys.path.insert(0, str(module_root))

from roading import OSMRoadNetwork
from traffic_rules import load_topo_setting

from pcd_overlay import PcdOverlay

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
        super().__init__("osm_nav_bridge")
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


class OSMMapEditor:
    """Mutable OSM node editor. Writes only after an explicit save request."""

    def __init__(self, map_path, pair_files_dir):
        self.map_path = Path(map_path).resolve()
        self.pair_files_dir = Path(pair_files_dir).resolve()
        self._lock = threading.Lock()

    def data(self, pcd_overlay, frame):
        if pcd_overlay is None:
            raise ValueError("PCD overlay is disabled")
        if frame not in {"raw", "aligned", "inverse"}:
            raise ValueError("frame must be raw, aligned, or inverse")
        root = ET.parse(self.map_path).getroot()
        nodes = {}
        for node in root.findall("node"):
            node_id = node.attrib.get("id")
            if not node_id:
                continue
            east, north = pcd_overlay.to_enu(node.attrib["lat"], node.attrib["lon"])
            nodes[node_id] = [east, north]
        roads, buildings, areas = [], [], []
        for way in root.findall("way"):
            refs = [nd.attrib["ref"] for nd in way.findall("nd") if nd.attrib.get("ref") in nodes]
            if len(refs) < 2:
                continue
            tags = {tag.attrib.get("k"): tag.attrib.get("v") for tag in way.findall("tag")}
            if tags.get("highway"):
                roads.append(refs)
            if len(refs) >= 3 and tags.get("building"):
                buildings.append(refs)
            if len(refs) >= 3 and (tags.get("landuse") or tags.get("natural")):
                areas.append(refs)
        signal_ids = []
        for node in root.findall("node"):
            tags = {tag.attrib.get("k"): tag.attrib.get("v") for tag in node.findall("tag")}
            if (tags.get("highway") == "traffic_signals" or tags.get("traffic_signals") == "signal"
                    or tags.get("crossing") == "traffic_signals" or tags.get("crossing:signals") == "yes"):
                if node.attrib.get("id") in nodes:
                    signal_ids.append(node.attrib["id"])
        return {
            "map_name": self.map_path.name,
            "origin": pcd_overlay.origin,
            "frame": frame,
            "points": pcd_overlay.editor_points(frame),
            "pcd_bounds": pcd_overlay.editor_bounds(frame),
            "nodes": [{"id": node_id, "east": point[0], "north": point[1]} for node_id, point in nodes.items()],
            "roads": roads,
            "buildings": buildings,
            "areas": areas,
            "signals": signal_ids,
        }

    def export_pairs(self, pcd_overlay, selections):
        """Write explicitly selected OSM/PCD correspondences as a new YAML file."""
        if pcd_overlay is None:
            raise ValueError("PCD overlay is disabled")
        if not isinstance(selections, list) or not selections:
            raise ValueError("selections must contain at least one OSM node")
        if len(selections) > 10_000:
            raise ValueError("a single export may contain at most 10000 pairs")
        if not self.pair_files_dir.is_dir():
            raise ValueError(f"pair files directory does not exist: {self.pair_files_dir}")
        with self._lock:
            root = ET.parse(self.map_path).getroot()
            known_ids = {node.attrib.get("id") for node in root.findall("node")}
            pairs, selected_ids = [], set()
            for index, selection in enumerate(selections):
                if not isinstance(selection, dict):
                    raise ValueError("each selected pair must be an object")
                node_id = str(selection.get("id", ""))
                if not node_id or node_id not in known_ids:
                    raise ValueError(f"unknown OSM node: {node_id}")
                if node_id in selected_ids:
                    raise ValueError(f"OSM node selected more than once: {node_id}")
                try:
                    east, north = float(selection["east"]), float(selection["north"])
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError(f"selected node {node_id} requires numeric East/North") from error
                if not math.isfinite(east) or not math.isfinite(north):
                    raise ValueError("selected node coordinates must be finite")
                latitude, longitude = pcd_overlay.to_lla(east, north)
                pairs.append({
                    "world": [east, north],
                    "gps": [latitude, longitude],
                })
                selected_ids.add(node_id)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            target = self.pair_files_dir / f"paired_{stamp}.yaml"
            suffix = 1
            while target.exists():
                target = self.pair_files_dir / f"paired_{stamp}_{suffix}.yaml"
                suffix += 1
            def number(value):
                text = f"{float(value):.9f}".rstrip("0").rstrip(".")
                return text if "." in text else text + ".0"

            lines = [
                "# Coordinate pairs for mode: 1.\n#\n",
                "# world is the SLAM/map coordinate in metres: [x, y]\n",
                "# gps is the WGS-84 geographic coordinate in degrees: [latitude, longitude]\n#\n",
                "# Enter at least three non-collinear, accurately surveyed pairs. Four or more\n",
                "# pairs are recommended because the affine transform is fitted by least squares.\n",
                "pairs:\n",
            ]
            for index, pair in enumerate(pairs):
                lines.extend((
                    f'  - id: "p{index}"\n',
                    f'    world: [{number(pair["world"][0])}, {number(pair["world"][1])}]\n',
                    f'    gps: [{number(pair["gps"][0])}, {number(pair["gps"][1])}]\n',
                ))
                if index != len(pairs) - 1:
                    lines.append("\n")
            content = "".join(lines)
            temporary = target.with_suffix(target.suffix + ".tmp")
            temporary.write_text(content, encoding="utf-8")
            temporary.replace(target)
        return {"pair_count": len(pairs), "file": target.name,
                "path": str(target), "mode_1_ready": len(pairs) >= 3}

    def save(self, pcd_overlay, edits):
        if not isinstance(edits, list) or not edits:
            raise ValueError("edits must contain at least one changed node")
        if len(edits) > 10000:
            raise ValueError("a single save may update at most 10000 nodes")
        with self._lock:
            tree = ET.parse(self.map_path)
            root = tree.getroot()
            node_by_id = {node.attrib.get("id"): node for node in root.findall("node")}
            updates = []
            for edit in edits:
                if not isinstance(edit, dict):
                    raise ValueError("each edit must be an object")
                node_id = str(edit.get("id", ""))
                if node_id not in node_by_id:
                    raise ValueError(f"unknown OSM node: {node_id}")
                east, north = float(edit["east"]), float(edit["north"])
                if not math.isfinite(east) or not math.isfinite(north):
                    raise ValueError("node coordinates must be finite")
                latitude, longitude = pcd_overlay.to_lla(east, north)
                updates.append((node_by_id[node_id], latitude, longitude))
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            backup = self.map_path.with_name(f"{self.map_path.stem}.{stamp}.bak{self.map_path.suffix}")
            suffix = 1
            while backup.exists():
                backup = self.map_path.with_name(f"{self.map_path.stem}.{stamp}.{suffix}.bak{self.map_path.suffix}")
                suffix += 1
            shutil.copy2(self.map_path, backup)
            for node, latitude, longitude in updates:
                node.set("lat", f"{latitude:.9f}")
                node.set("lon", f"{longitude:.9f}")
            ET.indent(tree, space="  ")
            temporary = self.map_path.with_suffix(self.map_path.suffix + ".tmp")
            tree.write(temporary, encoding="utf-8", xml_declaration=True)
            temporary.replace(self.map_path)
        return {"updated_nodes": len(updates), "backup": backup.name, "restart_required": True}

def default_map_path():
    return Path(get_package_share_directory("osm_nav")) / "maps" / "0924_4.osm"


def make_handler(network, web_dir, mode=0, ros_bridge=None, pcd_overlay=None, map_editor=None):
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
            if request.path == "/api/editor/data":
                try:
                    if map_editor is None:
                        raise ValueError("OSM editor is unavailable")
                    frame = parse_qs(request.query).get("frame", ["raw"])[0]
                    self.send_json(map_editor.data(pcd_overlay, frame))
                except ValueError as error:
                    self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
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
            if request.path == "/api/editor/export-pairs":
                try:
                    if map_editor is None:
                        raise ValueError("OSM editor is unavailable")
                    content_length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < content_length <= 2_000_000:
                        raise ValueError("pair export body must be between 1 and 2000000 bytes")
                    payload = json.loads(self.rfile.read(content_length))
                    if not isinstance(payload, dict):
                        raise ValueError("pair export request must be a JSON object")
                    self.send_json(map_editor.export_pairs(pcd_overlay, payload.get("selections")))
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                    self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
                return
            if request.path == "/api/editor/save":
                try:
                    if map_editor is None:
                        raise ValueError("OSM editor is unavailable")
                    content_length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < content_length <= 2_000_000:
                        raise ValueError("editor request body must be between 1 and 2000000 bytes")
                    payload = json.loads(self.rfile.read(content_length))
                    if not isinstance(payload, dict):
                        raise ValueError("editor request must be a JSON object")
                    self.send_json(map_editor.save(pcd_overlay, payload.get("edits")))
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                    self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
                return
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
    parser.add_argument("--topo-setting-yaml", type=Path, required=True,
                        help="Scenario profile YAML (topo_setting.yaml)")
    # Keep the spelling aligned with the existing osm_nav.yaml parameter.
    parser.add_argument("--topo-setting-chose", required=True,
                        help="Selected scene name, for example walk or car")
    parser.add_argument("--default-display-mode", choices=("compact", "full"), default="compact")
    parser.add_argument("--min-zoom-width", type=float, default=DEFAULT_MIN_ZOOM_WIDTH)
    parser.add_argument("--use-base-map", choices=("openstreetmap", "esri_satellite", "none"), default="esri_satellite")
    parser.add_argument("--mode", type=int, choices=(0, 1), default=0)
    parser.add_argument("--odometry-topic", default="/Odometry")
    parser.add_argument("--global-path-topic", default="/global_path")
    parser.add_argument("--slam-frame-id", default="map")
    parser.add_argument("--paired-yaml", type=Path)
    parser.add_argument("--stop-nav", default="/stop")
    parser.add_argument("--stop-override-topic", default="/cmd_vel/final_align")
    parser.add_argument("--signal-stop-distance-m", type=float, default=DEFAULT_SIGNAL_STOP_DISTANCE_M)
    parser.add_argument("--pcd-overlay", type=parse_bool, default=False)
    parser.add_argument("--pcd", default="", help="PCD map file path")
    parser.add_argument("--manifest-yaml", default="", help="PCD manifest YAML file path")
    parser.add_argument("--pair-files-dir", type=Path, required=True,
                        help="Directory for timestamped paired_*.yaml exports")
    args = parser.parse_args()
    if not args.map.is_file():
        parser.error(f"map file does not exist: {args.map}")
    if args.min_zoom_width <= 0 or args.signal_stop_distance_m <= 0:
        parser.error("minimum zoom width and signal stop distance must be positive")
    try:
        topo_setting = load_topo_setting(args.topo_setting_yaml, args.topo_setting_chose)
    except ValueError as error:
        parser.error(f"invalid topology setting: {error}")
    server = None

    network = OSMRoadNetwork(
        args.map, topo_setting["speed_mps"], topo_setting["signal_wait_seconds"],
        topo_setting["safety_multipliers"],
        {
            "default_display_mode": args.default_display_mode,
            "min_zoom_width": args.min_zoom_width,
            "use_base_map": args.use_base_map,
            "mode": args.mode,
            "global_path_topic": args.global_path_topic,
            "signal_stop_distance_m": args.signal_stop_distance_m,
            "topo_setting": {
                "name": topo_setting["name"],
                "speed_mps": topo_setting["speed_mps"],
                "allowed_road_types": sorted(topo_setting["allowed_road_types"]),
            },
        },
        allowed_road_types=topo_setting["allowed_road_types"],
    )
    pcd_overlay = None
    if args.pcd_overlay:
        if not args.pcd or not args.manifest_yaml:
            parser.error("--pcd and --manifest-yaml are required when --pcd-overlay is true")
        try:
            pcd_overlay = PcdOverlay(args.pcd, args.manifest_yaml)
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

    package_share = Path(get_package_share_directory("osm_nav"))
    network.ui_config["transform"] = transform_metadata
    try:
        server = ThreadingHTTPServer(
            (args.host, args.port), make_handler(network, package_share / "web", args.mode, ros_bridge, pcd_overlay, OSMMapEditor(args.map, args.pair_files_dir))
        )
    except OSError as error:
        if error.errno == errno.EADDRINUSE:
            parser.error(f"port {args.port} is already in use; close the existing server or choose --port <number>")
        raise
    print(f"Map: {args.map}")
    print(f"Topology setting: {topo_setting['name']} (speed {topo_setting['speed_mps']} m/s; "
          f"roads: {', '.join(sorted(topo_setting['allowed_road_types']))})")
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
