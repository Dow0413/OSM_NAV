#!/usr/bin/env python3
"""Render a Lanelet2 OSM map and optionally its shortest vehicle route."""

import argparse
from pathlib import Path

import lanelet2
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
from lanelet2.core import BasicPoint2d
from lanelet2.projection import UtmProjector


def default_map_path():
    return Path(get_package_share_directory("lanelet2_maps")) / "res" / "mapping_example.osm"


def boundary_xy(lanelet):
    left = list(lanelet.leftBound)
    right = list(lanelet.rightBound)
    return [p.x for p in left], [p.y for p in left], [p.x for p in right], [p.y for p in right]


def center_xy(lanelet):
    centerline = lanelet.centerline
    return [p.x for p in centerline], [p.y for p in centerline]


def nearest_drivable_lanelet(lanelet_map, traffic_rules, point):
    candidates = lanelet2.geometry.findNearest(
        lanelet_map.laneletLayer, BasicPoint2d(point[0], point[1]), 20)
    for distance, lanelet in candidates:
        if traffic_rules.canPass(lanelet):
            return lanelet, distance
    raise RuntimeError(f"No drivable lanelet was found near ({point[0]:.2f}, {point[1]:.2f}).")


def draw_map(ax, lanelet_map):
    for lanelet in lanelet_map.laneletLayer:
        lx, ly, rx, ry = boundary_xy(lanelet)
        ax.plot(lx, ly, color="0.70", linewidth=0.7, zorder=1)
        ax.plot(rx, ry, color="0.70", linewidth=0.7, zorder=1)


def draw_route(ax, path):
    for index, lanelet in enumerate(path):
        lx, ly, rx, ry = boundary_xy(lanelet)
        ax.fill(lx + list(reversed(rx)), ly + list(reversed(ry)),
                color="tab:orange", alpha=0.25, zorder=2)
        cx, cy = center_xy(lanelet)
        ax.plot(cx, cy, color="tab:red", linewidth=2.5, zorder=3,
                label="Shortest vehicle route" if index == 0 else None)

    sx, sy = center_xy(path[0])
    gx, gy = center_xy(path[-1])
    ax.scatter(sx[0], sy[0], c="tab:green", marker="o", s=80,
               edgecolors="black", label="Matched start", zorder=4)
    ax.scatter(gx[-1], gy[-1], c="tab:blue", marker="*", s=200,
               edgecolors="black", label="Matched goal", zorder=4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=default_map_path(),
                        help="Lanelet2 .osm map path (default: bundled mapping_example.osm)")
    parser.add_argument("--origin", type=float, nargs=2, metavar=("LAT", "LON"), default=(49.0, 8.4),
                        help="WGS84 projection origin, in degrees (default: 49.0 8.4)")
    parser.add_argument("--start", type=float, nargs=2, metavar=("X", "Y"),
                        help="Optional local-metre start coordinate")
    parser.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"),
                        help="Optional local-metre goal coordinate")
    parser.add_argument("--output", type=Path, default=Path("map_viz.png"),
                        help="PNG output path (default: map_viz.png)")
    parser.add_argument("--show", action="store_true", help="Open an interactive Matplotlib window")
    args = parser.parse_args()

    if (args.start is None) != (args.goal is None):
        parser.error("--start and --goal must be supplied together")
    if not args.map.is_file():
        parser.error(f"map file does not exist: {args.map}")

    projector = UtmProjector(lanelet2.io.Origin(args.origin[0], args.origin[1]))
    lanelet_map = lanelet2.io.load(str(args.map), projector)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
    draw_map(ax, lanelet_map)

    if args.start is not None:
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle)
        start, start_distance = nearest_drivable_lanelet(lanelet_map, traffic_rules, args.start)
        goal, goal_distance = nearest_drivable_lanelet(lanelet_map, traffic_rules, args.goal)
        route = lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules).getRoute(start, goal)
        if route is None:
            raise RuntimeError("No vehicle route exists between the matched lanelets.")
        path = list(route.shortestPath())
        draw_route(ax, path)
        print(f"Start: lanelet {start.id}, match distance {start_distance:.2f} m")
        print(f"Goal: lanelet {goal.id}, match distance {goal_distance:.2f} m")
        print("Route: " + " -> ".join(str(lanelet.id) for lanelet in path))

    ax.set_title("Lanelet2 OSM map" + (" and shortest vehicle route" if args.start else ""))
    ax.set_xlabel("local x [m]")
    ax.set_ylabel("local y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(linewidth=0.3, alpha=0.4)
    if args.start is not None:
        ax.legend()
    fig.savefig(args.output, dpi=180)
    print(f"Saved visualization: {args.output.resolve()}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
