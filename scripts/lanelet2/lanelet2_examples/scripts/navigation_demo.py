#!/usr/bin/env python3
"""Compute and visualize the shortest vehicle route in the Lanelet2 example map."""

import argparse
from pathlib import Path

import lanelet2
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
from lanelet2.projection import UtmProjector

START_ID = 4984315
GOAL_ID = 2925017


def boundary_xy(lanelet):
    left = list(lanelet.leftBound)
    right = list(lanelet.rightBound)
    return [p.x for p in left], [p.y for p in left], [p.x for p in right], [p.y for p in right]


def center_xy(lanelet):
    center = lanelet.centerline
    return [p.x for p in center], [p.y for p in center]


def plot_navigation(lanelet_map, path, output):
    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)

    for lanelet in lanelet_map.laneletLayer:
        lx, ly, rx, ry = boundary_xy(lanelet)
        ax.plot(lx, ly, color="0.70", linewidth=0.8, zorder=1)
        ax.plot(rx, ry, color="0.70", linewidth=0.8, zorder=1)

    for index, lanelet in enumerate(path):
        lx, ly, rx, ry = boundary_xy(lanelet)
        ax.fill(lx + list(reversed(rx)), ly + list(reversed(ry)),
                color="tab:orange", alpha=0.25, zorder=2)
        cx, cy = center_xy(lanelet)
        ax.plot(cx, cy, color="tab:red", linewidth=2.5, zorder=3,
                label="Shortest path" if index == 0 else None)

    start_x, start_y = center_xy(path[0])
    goal_x, goal_y = center_xy(path[-1])
    ax.scatter(start_x[0], start_y[0], c="tab:green", marker="o", s=80,
               edgecolors="black", label="Start", zorder=4)
    ax.scatter(goal_x[-1], goal_y[-1], c="tab:blue", marker="*", s=200,
               edgecolors="black", label="Goal", zorder=4)

    ax.set_title("Lanelet2 shortest-path navigation (vehicle)")
    ax.set_xlabel("local x [m]")
    ax.set_ylabel("local y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(linewidth=0.3, alpha=0.4)
    ax.legend()
    fig.savefig(output, dpi=180)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("navigation_demo.png"),
                        help="PNG output path (default: navigation_demo.png)")
    parser.add_argument("--show", action="store_true",
                        help="Also open an interactive Matplotlib window.")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    map_path = Path(get_package_share_directory("lanelet2_maps")) / "res" / "mapping_example.osm"
    projector = UtmProjector(lanelet2.io.Origin(49.0, 8.4))
    lanelet_map = lanelet2.io.load(str(map_path), projector)

    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules)
    route = graph.getRoute(lanelet_map.laneletLayer[START_ID],
                           lanelet_map.laneletLayer[GOAL_ID])
    if route is None:
        raise RuntimeError("The example map does not contain a vehicle route.")

    path = list(route.shortestPath())
    if not path:
        raise RuntimeError("The route contains no lanelets.")

    plot_navigation(lanelet_map, path, args.output)
    print(f"Route: {' -> '.join(str(lanelet.id) for lanelet in path)}")
    print(f"Saved visualization: {args.output.resolve()}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
