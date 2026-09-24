#!/usr/bin/env python3
"""Plan and render a shortest driving route on a standard OSM road network."""

import argparse
import heapq
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt

from raw_osm_viz import ROAD_STYLE, project, tags, way_coordinates

DRIVABLE_HIGHWAYS = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "service"}


def distance(point_a, point_b):
    return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])


def dijkstra(graph, start):
    costs = {start: 0.0}
    previous = {}
    queue = [(0.0, start)]
    while queue:
        cost, current = heapq.heappop(queue)
        if cost != costs[current]:
            continue
        for neighbor, edge_cost in graph.get(current, []):
            candidate = cost + edge_cost
            if candidate < costs.get(neighbor, float("inf")):
                costs[neighbor] = candidate
                previous[neighbor] = current
                heapq.heappush(queue, (candidate, neighbor))
    return costs, previous


def build_graph(root):
    geographic_nodes = {
        node.attrib["id"]: (float(node.attrib["lat"]), float(node.attrib["lon"]))
        for node in root.findall("node")
    }
    latitudes = [point[0] for point in geographic_nodes.values()]
    longitudes = [point[1] for point in geographic_nodes.values()]
    origin = ((min(latitudes) + max(latitudes)) / 2, (min(longitudes) + max(longitudes)) / 2)
    nodes = {node_id: project(lat, lon, *origin) for node_id, (lat, lon) in geographic_nodes.items()}

    graph = {}
    roads = []
    for way in root.findall("way"):
        way_tags = tags(way)
        if way_tags.get("highway") not in DRIVABLE_HIGHWAYS:
            continue
        references = [reference.attrib["ref"] for reference in way.findall("nd")]
        references = [node_id for node_id in references if node_id in nodes]
        if len(references) < 2:
            continue
        roads.append((way_coordinates(way, geographic_nodes, origin), way_tags))
        one_way = way_tags.get("oneway")
        pairs = list(zip(references, references[1:]))
        if one_way == "-1":
            pairs = [(end, start) for start, end in pairs]
        for start, end in pairs:
            edge_cost = distance(nodes[start], nodes[end])
            graph.setdefault(start, []).append((end, edge_cost))
            if one_way not in {"yes", "-1"}:
                graph.setdefault(end, []).append((start, edge_cost))
    return nodes, graph, roads


def automatic_endpoints(graph, nodes):
    candidates = list(graph)
    if len(candidates) < 2:
        raise RuntimeError("The OSM file has no connected drivable road network.")
    left = sorted(candidates, key=lambda node_id: nodes[node_id][0])[:20]
    right = sorted(candidates, key=lambda node_id: nodes[node_id][0], reverse=True)[:20]
    best = None
    for start in left:
        costs, previous = dijkstra(graph, start)
        for goal in right:
            if goal in costs and (best is None or costs[goal] > best[0]):
                best = (costs[goal], start, goal, previous)
    if best is None:
        raise RuntimeError("No directed route connects the automatically selected road endpoints.")
    return best


def reconstruct_path(previous, start, goal):
    path = [goal]
    while path[-1] != start:
        path.append(previous[path[-1]])
    return list(reversed(path))


def render(map_path, output, start_node=None, goal_node=None):
    root = ET.parse(map_path).getroot()
    nodes, graph, roads = build_graph(root)
    if start_node is None:
        route_length, start_node, goal_node, previous = automatic_endpoints(graph, nodes)
    else:
        if start_node not in graph or goal_node not in nodes:
            raise RuntimeError("The requested OSM node is not in the drivable road graph.")
        costs, previous = dijkstra(graph, start_node)
        if goal_node not in costs:
            raise RuntimeError("No directed route exists between the requested OSM nodes.")
        route_length = costs[goal_node]
    path = reconstruct_path(previous, start_node, goal_node)

    plt.rcParams["font.family"] = ["Noto Sans CJK SC", "sans-serif"]
    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
    ax.set_facecolor("#f5eef6")
    for coordinates, way_tags in roads:
        if len(coordinates) < 2:
            continue
        width, color = ROAD_STYLE.get(way_tags["highway"], (2.5, "#ffffff"))
        xs, ys = zip(*coordinates)
        ax.plot(xs, ys, color="#b8ad9f", linewidth=width + 1.2, zorder=1, solid_capstyle="round")
        ax.plot(xs, ys, color=color, linewidth=width, zorder=2, solid_capstyle="round")

    ax.scatter([point[0] for point in nodes.values()], [point[1] for point in nodes.values()],
               s=7, color="#35424c", alpha=0.75, zorder=3, label="OSM nodes")

    route_x = [nodes[node_id][0] for node_id in path]
    route_y = [nodes[node_id][1] for node_id in path]
    ax.plot(route_x, route_y, color="tab:red", linewidth=3.2, zorder=4, label="Dijkstra shortest route")
    ax.scatter(route_x[0], route_y[0], color="tab:green", marker="o", s=90,
               edgecolors="black", zorder=5, label="Start")
    ax.scatter(route_x[-1], route_y[-1], color="tab:blue", marker="*", s=220,
               edgecolors="black", zorder=5, label="Goal")
    ax.set_title(f"OSM driving-route demo: {map_path.name}")
    ax.set_aspect("equal", adjustable="box")
    ax.set_axis_off()
    ax.legend(loc="best")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, facecolor=fig.get_facecolor())

    print(f"Start OSM node: {start_node}")
    print(f"Goal OSM node: {goal_node}")
    print(f"Route length: {route_length:.1f} m across {len(path)} OSM nodes")
    print(f"Saved visualization: {output.resolve()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True, help="Standard OpenStreetMap .osm file")
    parser.add_argument("--output", type=Path, default=Path("osm_navigation.png"), help="PNG output path")
    parser.add_argument("--start-node", type=str, help="Optional OSM node ID for the route start")
    parser.add_argument("--goal-node", type=str, help="Optional OSM node ID for the route goal")
    args = parser.parse_args()
    if (args.start_node is None) != (args.goal_node is None):
        parser.error("--start-node and --goal-node must be supplied together")
    if not args.map.is_file():
        parser.error(f"map file does not exist: {args.map}")
    render(args.map, args.output, args.start_node, args.goal_node)


if __name__ == "__main__":
    main()
