"""Build the immutable pedestrian planning topology from an OSM XML map."""

import json
import math
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

from planner import shortest_path
from traffic_rules import (
    DEDICATED_PEDESTRIAN_WAYS, DEFAULT_ROBOT_DOG_SPEED_MPS,
    DEFAULT_SIGNAL_WAIT_SECONDS, SAFETY_MULTIPLIER, distance_m, is_walkable,
    planning_cost_per_meter, robot_dog_oneway,
)

DEFAULT_MIN_ZOOM_WIDTH = 2.0
DEFAULT_SIGNAL_STOP_DISTANCE_M = 2.0
EARTH_RADIUS_M = 6_371_000


def _tags(element):
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


class OSMRoadNetwork:
    """OSM display data plus a weighted, pedestrian-accessible road graph."""

    def __init__(self, map_path, robot_dog_speed_mps=DEFAULT_ROBOT_DOG_SPEED_MPS,
                 signal_wait_seconds=DEFAULT_SIGNAL_WAIT_SECONDS, safety_multipliers=None,
                 ui_config=None):
        self.map_path = Path(map_path)
        self.robot_dog_speed_mps = robot_dog_speed_mps
        self.signal_wait_seconds = signal_wait_seconds
        self.safety_multipliers = dict(SAFETY_MULTIPLIER)
        self.safety_multipliers.update(safety_multipliers or {})
        self.ui_config = ui_config or {}
        root = ET.parse(map_path).getroot()
        self.nodes = {n.attrib["id"]: (float(n.attrib["lat"]), float(n.attrib["lon"])) for n in root.findall("node")}
        self.traffic_signal_nodes = {n.attrib["id"] for n in root.findall("node") if _tags(n).get("highway") == "traffic_signals" or _tags(n).get("traffic_signals") == "signal" or _tags(n).get("crossing") == "traffic_signals" or _tags(n).get("crossing:signals") == "yes"}
        self.signal_coordinates = {node_id: self.nodes[node_id] for node_id in self.traffic_signal_nodes}
        self._signal_lock = threading.Lock()
        self.signal_states = {node_id: 1 for node_id in self.traffic_signal_nodes}
        self.signal_crossing_count = {node_id: 0 for node_id in self.traffic_signal_nodes}
        if not self.nodes:
            raise RuntimeError("The OSM file has no nodes.")
        lats, lons = zip(*self.nodes.values())
        self.bounds = {"min_lat": min(lats), "min_lon": min(lons), "max_lat": max(lats), "max_lon": max(lons)}
        self.reference_lat = sum(lats) / len(lats)
        self.graph, self.segments, self.segment_by_edge = {}, [], {}
        self.roads, self.buildings, self.areas = [], [], []
        self._build_topology(root)
        self._map_payloads = {"compact": self._build_map_data("compact"), "full": self._build_map_data("full")}
        self._map_json = {k: json.dumps(v, ensure_ascii=False, separators=(",", ":")).encode("utf-8") for k, v in self._map_payloads.items()}

    def _build_topology(self, root):
        for way in root.findall("way"):
            way_tags = _tags(way)
            references = [ref.attrib["ref"] for ref in way.findall("nd") if ref.attrib["ref"] in self.nodes]
            coordinates = [[*self.nodes[node_id]] for node_id in references]
            if len(coordinates) >= 3 and way_tags.get("building"):
                self.buildings.append({"coordinates": coordinates, "name": way_tags.get("name", "")})
            area_kind = way_tags.get("landuse") or way_tags.get("natural")
            if len(coordinates) >= 3 and area_kind:
                self.areas.append({"coordinates": coordinates, "kind": area_kind, "name": way_tags.get("name", "")})
            road_type = way_tags.get("highway")
            if road_type:
                self.roads.append({"coordinates": coordinates, "kind": road_type, "name": way_tags.get("name", ""), "oneway": way_tags.get("oneway", ""), "lanes": way_tags.get("lanes", ""), "maxspeed": way_tags.get("maxspeed", "")})
            if not is_walkable(road_type, way_tags) or len(references) < 2:
                continue
            one_way = robot_dog_oneway(road_type, way_tags)
            pairs = list(zip(references, references[1:]))
            way_signals = [node_id for node_id in references if node_id in self.traffic_signal_nodes]
            synthetic_signal_id = None
            if road_type in DEDICATED_PEDESTRIAN_WAYS and not way_signals and way_tags.get("crossing") == "traffic_signals":
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
                start, end = (original_end, original_start) if one_way == "-1" else (original_start, original_end)
                length = distance_m(self.nodes[start], self.nodes[end])
                cost_per_m = planning_cost_per_meter(road_type, self.robot_dog_speed_mps, self.safety_multipliers) + signal_wait_per_segment / length
                cost = length * cost_per_m
                self.graph.setdefault(start, []).append((end, cost))
                bidirectional = one_way not in {"yes", "-1"}
                if bidirectional:
                    self.graph.setdefault(end, []).append((start, cost))
                segment = {"start": start, "end": end, "bidirectional": bidirectional, "kind": road_type, "name": way_tags.get("name", ""), "cost_per_m": cost_per_m, "signal_controlled": signal_controlled, "signal_id": signal_id}
                self.segments.append(segment)
                for edge in ([(start, end), (end, start)] if bidirectional else [(start, end)]):
                    if edge not in self.segment_by_edge or signal_controlled:
                        self.segment_by_edge[edge] = segment

    def map_data(self, detail="compact"):
        if detail not in {"compact", "full"}: raise ValueError("map detail must be compact or full")
        return self._map_payloads[detail]

    def map_json(self, detail="compact"):
        if detail not in {"compact", "full"}: raise ValueError("map detail must be compact or full")
        return self._map_json[detail]

    def _build_map_data(self, detail):
        if detail == "compact":
            return {"bounds": self.bounds, "detail": "compact", "roads": [{"coordinates": r["coordinates"], "kind": r["kind"], "oneway": r["oneway"]} for r in self.roads if len(r["coordinates"]) >= 2]}
        return {"bounds": self.bounds, "detail": "full", "roads": self.roads, "buildings": self.buildings, "areas": self.areas, "nodes": [[i, *p] for i, p in self.nodes.items()], "traffic_signals": [[i, *self.nodes[i]] for i in sorted(self.traffic_signal_nodes)]}

    def config_data(self):
        return {"map_name": self.map_path.name, "default_display_mode": self.ui_config.get("default_display_mode", "compact"), "min_zoom_width": self.ui_config.get("min_zoom_width", DEFAULT_MIN_ZOOM_WIDTH), "use_base_map": self.ui_config.get("use_base_map", "esri_satellite"), "mode": self.ui_config.get("mode", 0), "global_path_topic": self.ui_config.get("global_path_topic", "/global_path"), "transform": self.ui_config.get("transform"), "signal_stop_distance_m": self.ui_config.get("signal_stop_distance_m", DEFAULT_SIGNAL_STOP_DISTANCE_M)}

    def signal_state(self, signal_id):
        with self._signal_lock: return self.signal_states[signal_id]

    def set_signal_state(self, signal_id, state):
        if type(state) is not int or state not in (0, 1): raise ValueError("signal state must be 0 (green) or 1 (red)")
        with self._signal_lock:
            if signal_id not in self.signal_states: raise KeyError(signal_id)
            self.signal_states[signal_id] = state

    def signals_data(self):
        with self._signal_lock:
            return [{"id": i, "latitude": p[0], "longitude": p[1], "state": self.signal_states[i], "controls_pedestrian_crossing": self.signal_crossing_count[i] > 0} for i, p in sorted(self.signal_coordinates.items())]

    def _to_local(self, point):
        return (math.radians(point[1]) * EARTH_RADIUS_M * math.cos(math.radians(self.reference_lat)), math.radians(point[0]) * EARTH_RADIUS_M)

    def _project_to_segment(self, point, start, end):
        px, py = self._to_local(point); ax, ay = self._to_local(start); bx, by = self._to_local(end)
        dx, dy = bx - ax, by - ay; length_sq = dx * dx + dy * dy
        fraction = 0.0 if not length_sq else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        projected = (start[0] + (end[0] - start[0]) * fraction, start[1] + (end[1] - start[1]) * fraction)
        return fraction, projected, distance_m(point, projected)

    def snap_to_road(self, point):
        if not self.segments: raise ValueError("The OSM map has no walkable road segments for the robot-dog profile.")
        best = min(((self._project_to_segment(point, self.nodes[s["start"]], self.nodes[s["end"]]), s) for s in self.segments), key=lambda x: x[0][2])
        (fraction, coordinate, snap_distance), segment = best
        return {"coordinate": coordinate, "distance_m": snap_distance, "fraction": fraction, "segment": segment, "road": {"name": segment["name"], "kind": segment["kind"]}}

    def _graph_with_virtual_points(self, start_snap, goal_snap):
        graph, coordinates = {n: list(e) for n, e in self.graph.items()}, dict(self.nodes)
        def attach(key, snap):
            segment = snap["segment"]; start, end = segment["start"], segment["end"]; coordinate = snap["coordinate"]
            coordinates[key] = coordinate; graph.setdefault(key, [])
            first = distance_m(self.nodes[start], coordinate) * segment["cost_per_m"]; second = distance_m(coordinate, self.nodes[end]) * segment["cost_per_m"]
            graph.setdefault(start, []).append((key, first)); graph[key].append((end, second))
            if segment["bidirectional"]: graph.setdefault(end, []).append((key, second)); graph[key].append((start, first))
        attach("__start__", start_snap); attach("__goal__", goal_snap)
        return graph, coordinates

    def route(self, start_point, goal_point):
        start_snap, goal_snap = self.snap_to_road(start_point), self.snap_to_road(goal_point)
        graph, coordinates = self._graph_with_virtual_points(start_snap, goal_snap)
        planning_cost, path = shortest_path(graph, "__start__", "__goal__")
        path_coordinates = [[*coordinates[n]] for n in path]
        distance = sum(distance_m(a, b) for a, b in zip(path_coordinates, path_coordinates[1:]))
        events = []
        for index, (a, b) in enumerate(zip(path, path[1:])):
            segment = start_snap["segment"] if a == "__start__" else goal_snap["segment"] if b == "__goal__" else self.segment_by_edge.get((a, b))
            signal_id = segment["signal_id"] if segment else None
            if signal_id:
                if events and events[-1]["signal_id"] == signal_id and events[-1]["exit_index"] == index: events[-1]["exit_index"] = index + 1
                else: events.append({"signal_id": signal_id, "entry_index": index, "exit_index": index + 1})
        return {"path": path_coordinates, "distance_m": distance, "planning_cost_s": planning_cost, "traffic_signal_count": len(events), "signal_wait_seconds": len(events) * self.signal_wait_seconds, "crossing_events": events, "start": start_snap, "goal": goal_snap}
