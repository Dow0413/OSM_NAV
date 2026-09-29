"""Build the immutable pedestrian planning topology from an OSM XML map."""

import json
import math
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

from planner import shortest_path_with_turn_restrictions
from traffic_rules import DEFAULT_SIGNAL_WAIT_SECONDS, distance_m

DEFAULT_PLANNING_SPEED_MPS = 1.0
DEFAULT_MIN_ZOOM_WIDTH = 2.0
DEFAULT_SIGNAL_STOP_DISTANCE_M = 2.0
EARTH_RADIUS_M = 6_371_000


def _tags(element):
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


class OSMRoadNetwork:
    """OSM display data plus a weighted graph filtered by topology and rules."""

    def __init__(self, map_path, robot_dog_speed_mps=DEFAULT_PLANNING_SPEED_MPS,
                 signal_wait_seconds=DEFAULT_SIGNAL_WAIT_SECONDS, safety_multipliers=None,
                 ui_config=None, allowed_road_types=None, traffic_rules=None):
        self.map_path = Path(map_path)
        self.robot_dog_speed_mps = robot_dog_speed_mps
        self.signal_wait_seconds = signal_wait_seconds
        self.safety_multipliers = dict(safety_multipliers or {})
        self.allowed_road_types = set(allowed_road_types) if allowed_road_types else None
        if traffic_rules is None:
            raise ValueError("OSMRoadNetwork requires a traffic-rules object")
        self.traffic_rules = traffic_rules
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
        self.graph, self.transitions = {}, {}
        self.segments, self.segment_by_edge, self.segment_by_traversal = [], {}, {}
        self.roads, self.buildings, self.areas = [], [], []
        self.turn_restrictions = self._read_turn_restrictions(root)
        self._build_topology(root)
        self._map_payloads = {"compact": self._build_map_data("compact"), "full": self._build_map_data("full")}
        self._map_json = {k: json.dumps(v, ensure_ascii=False, separators=(",", ":")).encode("utf-8") for k, v in self._map_payloads.items()}

    @staticmethod
    def _read_turn_restrictions(root):
        """Read standard OSM ``from way / via node / to way`` restrictions."""
        restrictions = {}
        for relation in root.findall("relation"):
            tags = _tags(relation)
            restriction = tags.get("restriction", "")
            if tags.get("type") != "restriction" or not restriction:
                continue
            members = {member.attrib.get("role"): member for member in relation.findall("member")}
            from_member, via_member, to_member = (members.get(role) for role in ("from", "via", "to"))
            if (from_member is None or to_member is None or via_member is None
                    or from_member.attrib.get("type") != "way"
                    or to_member.attrib.get("type") != "way"
                    or via_member.attrib.get("type") != "node"):
                continue
            via_id = via_member.attrib.get("ref")
            from_id, to_id = from_member.attrib.get("ref"), to_member.attrib.get("ref")
            if not via_id or not from_id or not to_id:
                continue
            entry = restrictions.setdefault(via_id, {"forbidden": set(), "only": {}})
            if restriction.startswith("only_"):
                entry["only"].setdefault(from_id, set()).add(to_id)
            elif restriction.startswith("no_"):
                entry["forbidden"].add((from_id, to_id))
        return restrictions

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
                self.roads.append({"id": way.attrib["id"], "coordinates": coordinates, "kind": road_type, "name": way_tags.get("name", ""), "oneway": way_tags.get("oneway", ""), "lanes": way_tags.get("lanes", ""), "maxspeed": way_tags.get("maxspeed", "")})
            if (not self.traffic_rules.can_pass(road_type, way_tags)
                    or (self.allowed_road_types is not None and road_type not in self.allowed_road_types)
                    or len(references) < 2):
                continue
            one_way = self.traffic_rules.one_way(road_type, way_tags)
            pairs = list(zip(references, references[1:]))
            way_signals = [node_id for node_id in references if node_id in self.traffic_signal_nodes]
            synthetic_signal_id = None
            if self.traffic_rules.creates_synthetic_signal(road_type, way_tags) and not way_signals:
                synthetic_signal_id = f"way:{way.attrib['id']}"
                self.signal_coordinates[synthetic_signal_id] = coordinates[len(coordinates) // 2]
                self.signal_states[synthetic_signal_id] = 1
                self.signal_crossing_count[synthetic_signal_id] = 0
            signal_controlled = self.traffic_rules.signal_controls_way(
                road_type, way_tags, way_signals or ([synthetic_signal_id] if synthetic_signal_id else [])
            )
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
                speed_limit_mps = self.traffic_rules.speed_limit_mps(way_tags, self.robot_dog_speed_mps)
                cost_per_m = self.safety_multipliers[road_type] / speed_limit_mps + signal_wait_per_segment / length
                cost = length * cost_per_m
                self.graph.setdefault(start, []).append((end, cost))
                way_id = way.attrib["id"]
                self.transitions.setdefault(start, []).append((end, cost, way_id))
                bidirectional = one_way not in {"yes", "-1"}
                if bidirectional:
                    self.graph.setdefault(end, []).append((start, cost))
                    self.transitions.setdefault(end, []).append((start, cost, way_id))
                segment = {"start": start, "end": end, "bidirectional": bidirectional, "kind": road_type, "name": way_tags.get("name", ""), "cost_per_m": cost_per_m, "signal_controlled": signal_controlled, "signal_id": signal_id,
                           "speed_limit_mps": speed_limit_mps, "way_id": way.attrib["id"]}
                self.segments.append(segment)
                for edge in ([(start, end), (end, start)] if bidirectional else [(start, end)]):
                    if edge not in self.segment_by_edge or signal_controlled:
                        self.segment_by_edge[edge] = segment
                    self.segment_by_traversal[(edge[0], edge[1], way_id)] = segment

    def map_data(self, detail="compact"):
        if detail not in {"compact", "full"}: raise ValueError("map detail must be compact or full")
        return self._map_payloads[detail]

    def map_json(self, detail="compact"):
        if detail not in {"compact", "full"}: raise ValueError("map detail must be compact or full")
        return self._map_json[detail]

    def _build_map_data(self, detail):
        if detail == "compact":
            return {"bounds": self.bounds, "detail": "compact", "roads": [{"id": r["id"], "coordinates": r["coordinates"], "kind": r["kind"], "oneway": r["oneway"]} for r in self.roads if len(r["coordinates"]) >= 2]}
        return {"bounds": self.bounds, "detail": "full", "roads": self.roads, "buildings": self.buildings, "areas": self.areas, "nodes": [[i, *p] for i, p in self.nodes.items()], "traffic_signals": [[i, *self.nodes[i]] for i in sorted(self.traffic_signal_nodes)]}

    def config_data(self):
        return {"map_name": self.map_path.name, "default_display_mode": self.ui_config.get("default_display_mode", "compact"), "min_zoom_width": self.ui_config.get("min_zoom_width", DEFAULT_MIN_ZOOM_WIDTH), "use_base_map": self.ui_config.get("use_base_map", "esri_satellite"), "mode": self.ui_config.get("mode", 0), "global_path_topic": self.ui_config.get("global_path_topic", "/global_path"), "transform": self.ui_config.get("transform"), "signal_stop_distance_m": self.ui_config.get("signal_stop_distance_m", DEFAULT_SIGNAL_STOP_DISTANCE_M), "topo_setting": self.ui_config.get("topo_setting")}

    def signal_state(self, signal_id):
        with self._signal_lock: return self.signal_states[signal_id]

    def set_signal_state(self, signal_id, state):
        if type(state) is not int or state not in (0, 1): raise ValueError("signal state must be 0 (green) or 1 (red)")
        with self._signal_lock:
            if signal_id not in self.signal_states: raise KeyError(signal_id)
            self.signal_states[signal_id] = state

    def signals_data(self):
        with self._signal_lock:
            return [{"id": i, "latitude": p[0], "longitude": p[1], "state": self.signal_states[i], "controls_active_profile": self.signal_crossing_count[i] > 0} for i, p in sorted(self.signal_coordinates.items())]

    def _to_local(self, point):
        return (math.radians(point[1]) * EARTH_RADIUS_M * math.cos(math.radians(self.reference_lat)), math.radians(point[0]) * EARTH_RADIUS_M)

    def _project_to_segment(self, point, start, end):
        px, py = self._to_local(point); ax, ay = self._to_local(start); bx, by = self._to_local(end)
        dx, dy = bx - ax, by - ay; length_sq = dx * dx + dy * dy
        fraction = 0.0 if not length_sq else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        projected = (start[0] + (end[0] - start[0]) * fraction, start[1] + (end[1] - start[1]) * fraction)
        return fraction, projected, distance_m(point, projected)

    def snap_to_road(self, point):
        if not self.segments: raise ValueError("The OSM map has no road segments permitted by the active topology and traffic rules.")
        best = min(((self._project_to_segment(point, self.nodes[s["start"]], self.nodes[s["end"]]), s) for s in self.segments), key=lambda x: x[0][2])
        (fraction, coordinate, snap_distance), segment = best
        return {"coordinate": coordinate, "distance_m": snap_distance, "fraction": fraction, "segment": segment, "road": {"name": segment["name"], "kind": segment["kind"]}}

    def _transitions_with_virtual_points(self, start_snap, goal_snap):
        transitions = {node: list(edges) for node, edges in self.transitions.items()}
        coordinates = dict(self.nodes)
        start_segment, goal_segment = start_snap["segment"], goal_snap["segment"]
        start_key, goal_key = "__start__", "__goal__"
        coordinates[start_key], coordinates[goal_key] = start_snap["coordinate"], goal_snap["coordinate"]

        def split_costs(snap):
            segment = snap["segment"]
            first = distance_m(self.nodes[segment["start"]], snap["coordinate"]) * segment["cost_per_m"]
            second = distance_m(snap["coordinate"], self.nodes[segment["end"]]) * segment["cost_per_m"]
            return segment, first, second

        segment, first, second = split_costs(start_snap)
        transitions[start_key] = [(segment["end"], second, segment["way_id"])]
        if segment["bidirectional"]:
            transitions[start_key].append((segment["start"], first, segment["way_id"]))

        segment, first, second = split_costs(goal_snap)
        transitions.setdefault(segment["start"], []).append((goal_key, first, segment["way_id"]))
        if segment["bidirectional"]:
            transitions.setdefault(segment["end"], []).append((goal_key, second, segment["way_id"]))

        # If both projected points lie on one directed segment, they must be
        # connected directly.  Without this edge Dijkstra can only leave the
        # start projection through a segment endpoint and then return from an
        # endpoint to the goal projection, producing an obvious detour.
        same_segment = (start_segment["way_id"] == goal_segment["way_id"]
                        and start_segment["start"] == goal_segment["start"]
                        and start_segment["end"] == goal_segment["end"])
        forward = goal_snap["fraction"] >= start_snap["fraction"]
        if same_segment and (start_segment["bidirectional"] or forward):
            direct_cost = distance_m(start_snap["coordinate"], goal_snap["coordinate"]) * start_segment["cost_per_m"]
            transitions[start_key].append((goal_key, direct_cost, start_segment["way_id"]))
        return transitions, coordinates

    def route(self, start_point, goal_point):
        start_snap, goal_snap = self.snap_to_road(start_point), self.snap_to_road(goal_point)
        transitions, coordinates = self._transitions_with_virtual_points(start_snap, goal_snap)
        planning_cost, state_path = shortest_path_with_turn_restrictions(
            transitions, "__start__", "__goal__",
            lambda from_way, via_node, to_way: self.traffic_rules.turn_allowed(
                self.turn_restrictions, from_way, via_node, to_way),
        )
        path = [state if isinstance(state, str) else state[0] for state in state_path]
        path_coordinates = [[*coordinates[n]] for n in path]
        distance = sum(distance_m(a, b) for a, b in zip(path_coordinates, path_coordinates[1:]))
        events, route_speeds = [], []
        for index, (a, b) in enumerate(zip(state_path, state_path[1:])):
            segment = (start_snap["segment"] if a == "__start__" else goal_snap["segment"] if b == "__goal__"
                       else self.segment_by_traversal.get((a[0], b[0], b[1])))
            if segment:
                route_speeds.append(segment["speed_limit_mps"])
            signal_id = segment["signal_id"] if segment else None
            if signal_id:
                if events and events[-1]["signal_id"] == signal_id and events[-1]["exit_index"] == index: events[-1]["exit_index"] = index + 1
                else: events.append({"signal_id": signal_id, "entry_index": index, "exit_index": index + 1})
        return {"path": path_coordinates, "distance_m": distance, "planning_cost_s": planning_cost, "traffic_signal_count": len(events), "signal_wait_seconds": len(events) * self.signal_wait_seconds, "crossing_events": events, "speed_limit_mps": min(route_speeds, default=self.robot_dog_speed_mps), "start": start_snap, "goal": goal_snap}
