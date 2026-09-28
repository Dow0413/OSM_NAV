"""Single configurable traffic-rule interpreter for every topology scenario."""
import math
import re

DEFAULT_SIGNAL_WAIT_SECONDS = 20.0


class TrafficRules:
    """Interpret speed, signal and direction tags according to traffic_info."""
    def __init__(self, traffic_info):
        self.info = dict(traffic_info)

    @property
    def profile(self):
        return self.info["name"]

    def can_pass(self, road_type, way_tags):
        # Topology settings are the sole authority for road eligibility.
        return bool(road_type)

    def one_way(self, road_type, way_tags):
        if not self.info["road_limit"]:
            return None
        value = way_tags.get("oneway")
        return value if value in {"yes", "-1"} else None

    def turn_allowed(self, restrictions, from_way_id, via_node_id, to_way_id):
        """Apply OSM turn relations only when road-direction rules are enabled."""
        if not self.info["road_limit"] or not from_way_id or from_way_id == to_way_id:
            return True
        at_node = restrictions.get(via_node_id)
        if not at_node:
            return True
        if (from_way_id, to_way_id) in at_node["forbidden"]:
            return False
        allowed_targets = at_node["only"].get(from_way_id)
        return not allowed_targets or to_way_id in allowed_targets

    def creates_synthetic_signal(self, road_type, way_tags):
        return self.info["traffic_signal_limit"] and way_tags.get("traffic_signals") in {"yes", "signal"}

    def signal_controls_way(self, road_type, way_tags, signal_ids):
        return self.info["traffic_signal_limit"] and bool(signal_ids)

    def speed_limit_mps(self, way_tags, desired_speed_mps):
        if not self.info["max_speed_limit"]:
            return desired_speed_mps
        value = str(way_tags.get("maxspeed", "")).strip().lower()
        match = re.fullmatch(r"(\d+(?:\.\d+)?)\s*(km/h|kph|kmh|mph)?", value)
        if not match:
            return desired_speed_mps
        speed = float(match.group(1))
        return min(desired_speed_mps, speed / 3.6 if match.group(2) != "mph" else speed * 0.44704)


def distance_m(a, b):
    """Approximate WGS-84 distance for short OSM road segments."""
    earth_radius_m = 6_371_000
    latitude = math.radians((a[0] + b[0]) / 2)
    dx = math.radians(a[1] - b[1]) * earth_radius_m * math.cos(latitude)
    dy = math.radians(a[0] - b[0]) * earth_radius_m
    return math.hypot(dx, dy)
