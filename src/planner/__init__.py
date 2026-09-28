"""Path-planning algorithms used by osm_nav."""

from .dijkstra import shortest_path, shortest_path_with_turn_restrictions

__all__ = ["shortest_path", "shortest_path_with_turn_restrictions"]
