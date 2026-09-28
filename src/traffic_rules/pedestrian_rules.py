"""Pedestrian-like robot access, direction, and traversal-cost rules."""

import math

WALKABLE = {
    "footway", "path", "pedestrian", "steps", "living_street", "residential", "service",
    "unclassified", "tertiary", "secondary", "primary", "track",
}
DEDICATED_PEDESTRIAN_WAYS = {"footway", "path", "pedestrian", "steps"}
SAFETY_MULTIPLIER = {
    "footway": 1.0, "pedestrian": 1.0, "path": 1.1, "steps": 1.5,
    "living_street": 1.2, "service": 1.3, "residential": 1.5, "unclassified": 1.8,
    "track": 1.8, "tertiary": 2.5, "secondary": 3.0, "primary": 4.0,
}
DEFAULT_ROBOT_DOG_SPEED_MPS = 1.0
DEFAULT_SIGNAL_WAIT_SECONDS = 20.0
EARTH_RADIUS_M = 6_371_000


def is_walkable(road_type, way_tags):
    return road_type in WALKABLE and way_tags.get("foot") not in {"no", "use_sidepath"} and way_tags.get("access") not in {"no", "private"}


def robot_dog_oneway(road_type, way_tags):
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
