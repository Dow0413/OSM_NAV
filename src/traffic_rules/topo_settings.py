"""Load and validate scenario-specific planning topology settings."""

from pathlib import Path
import math

import yaml

from .general_rules import DEFAULT_SIGNAL_WAIT_SECONDS


def load_topo_setting(setting_yaml, choice):
    """Return one validated profile from ``topo_setting.yaml``.

    The profile's ``safety_multipliers`` keys are also the road types admitted
    into the planning graph. A ``car`` scene therefore cannot route across
    pedestrian-only ways just because they are present in the OSM data.
    """
    path = Path(setting_yaml)
    if not path.is_file():
        raise ValueError(f"topology setting YAML does not exist: {path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as error:
        raise ValueError(f"cannot parse topology setting YAML: {error}") from error
    if not isinstance(document, dict):
        raise ValueError("topology setting YAML must contain a mapping of scene names")
    if choice not in document:
        available = ", ".join(map(str, document)) or "none"
        raise ValueError(f"topology setting '{choice}' does not exist (available: {available})")
    profile = document[choice]
    if not isinstance(profile, dict):
        raise ValueError(f"topology setting '{choice}' must be a mapping")
    info_choice = profile.get("traffic_info_chose")
    if not isinstance(info_choice, str) or not info_choice.strip():
        raise ValueError(f"topology setting '{choice}' requires traffic_info_chose")
    try:
        speed_mps = float(profile["speed_mps"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"topology setting '{choice}' requires numeric speed_mps") from error
    if not math.isfinite(speed_mps) or speed_mps <= 0:
        raise ValueError(f"topology setting '{choice}'.speed_mps must be positive")
    multipliers = profile.get("safety_multipliers")
    if not isinstance(multipliers, dict) or not multipliers:
        raise ValueError(f"topology setting '{choice}' requires a non-empty safety_multipliers mapping")
    try:
        multipliers = {str(kind): float(value) for kind, value in multipliers.items()}
    except (TypeError, ValueError) as error:
        raise ValueError(f"topology setting '{choice}' has a non-numeric road multiplier") from error
    if any(not kind or not math.isfinite(value) or value <= 0 for kind, value in multipliers.items()):
        raise ValueError(f"topology setting '{choice}' road multipliers must be positive finite values")
    try:
        signal_wait_seconds = float(profile.get("signal_wait_seconds", DEFAULT_SIGNAL_WAIT_SECONDS))
    except (TypeError, ValueError) as error:
        raise ValueError(f"topology setting '{choice}'.signal_wait_seconds must be numeric") from error
    if not math.isfinite(signal_wait_seconds) or signal_wait_seconds < 0:
        raise ValueError(f"topology setting '{choice}'.signal_wait_seconds cannot be negative")
    return {
        "name": str(choice),
        "traffic_info_chose": info_choice,
        "speed_mps": speed_mps,
        "safety_multipliers": multipliers,
        "allowed_road_types": frozenset(multipliers),
        "signal_wait_seconds": signal_wait_seconds,
    }
