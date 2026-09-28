"""Load generic traffic-rule switches and ROS topic names."""
from pathlib import Path
import yaml


def load_traffic_info(path, choice):
    path = Path(path)
    if not path.is_file():
        raise ValueError(f"traffic info YAML does not exist: {path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as error:
        raise ValueError(f"cannot parse traffic info YAML: {error}") from error
    profile = document.get(choice) if isinstance(document, dict) else None
    if not isinstance(profile, dict):
        available = ", ".join(document) if isinstance(document, dict) else "none"
        raise ValueError(f"traffic info '{choice}' does not exist (available: {available})")
    result = {"name": choice}
    for key in ("max_speed_limit", "traffic_signal_limit", "road_limit"):
        value = profile.get(key, False)
        if not isinstance(value, bool):
            raise ValueError(f"traffic info '{choice}.{key}' must be true or false")
        result[key] = value
    for key in ("max_speed", "wait_traffic_signal"):
        value = profile.get(key, "")
        if not isinstance(value, str):
            raise ValueError(f"traffic info '{choice}.{key}' must be a topic string")
        result[key] = value
    if result["max_speed_limit"] and not result["max_speed"]:
        raise ValueError(f"traffic info '{choice}' enables max_speed_limit but has no max_speed topic")
    if result["traffic_signal_limit"] and not result["wait_traffic_signal"]:
        raise ValueError(f"traffic info '{choice}' enables traffic_signal_limit but has no wait_traffic_signal topic")
    return result
