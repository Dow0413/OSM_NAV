#!/usr/bin/env python3
"""Render a standard OpenStreetMap .osm file to a labeled PNG map."""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt

EARTH_RADIUS_M = 6_371_000.0
ROAD_STYLE = {
    "motorway": (6.0, "#f4d98b"), "trunk": (5.5, "#f4d98b"),
    "primary": (5.0, "#f4e99e"), "secondary": (4.5, "#f4f59f"),
    "tertiary": (3.5, "#fff8bd"), "unclassified": (2.8, "#ffffff"),
    "residential": (2.5, "#ffffff"), "service": (1.8, "#ffffff"),
    "footway": (1.2, "#ffffff"), "path": (1.0, "#ffffff"),
}


def tags(element):
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


def project(lat, lon, origin_lat, origin_lon):
    x = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    y = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return x, y


def way_coordinates(way, nodes, origin):
    coordinates = []
    for reference in way.findall("nd"):
        node = nodes.get(reference.attrib["ref"])
        if node is not None:
            coordinates.append(project(node[0], node[1], *origin))
    return coordinates


def polygon(ax, coordinates, color, alpha):
    if len(coordinates) >= 3:
        xs, ys = zip(*coordinates)
        ax.fill(xs, ys, color=color, alpha=alpha, linewidth=0.4,
                edgecolor="#9a8f87", zorder=1)


def label(ax, coordinates, text, color="#505050", fontsize=8):
    if not text or not coordinates:
        return
    x = sum(point[0] for point in coordinates) / len(coordinates)
    y = sum(point[1] for point in coordinates) / len(coordinates)
    ax.text(x, y, text, ha="center", va="center", color=color,
            fontsize=fontsize, zorder=5)


def render(map_path, output):
    root = ET.parse(map_path).getroot()
    nodes = {
        node.attrib["id"]: (float(node.attrib["lat"]), float(node.attrib["lon"]))
        for node in root.findall("node")
    }
    if not nodes:
        raise RuntimeError("The OSM file contains no geographic nodes.")

    latitudes = [node[0] for node in nodes.values()]
    longitudes = [node[1] for node in nodes.values()]
    origin = ((min(latitudes) + max(latitudes)) / 2, (min(longitudes) + max(longitudes)) / 2)
    ways = [(way_coordinates(way, nodes, origin), tags(way)) for way in root.findall("way")]

    plt.rcParams["font.family"] = ["Noto Sans CJK SC", "sans-serif"]
    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
    ax.set_facecolor("#f5eef6")

    for coordinates, way_tags in ways:
        if "landuse" in way_tags or way_tags.get("leisure") == "park":
            polygon(ax, coordinates, "#e1cce6", 0.55)
            label(ax, coordinates, way_tags.get("name"), color="#765f77")

    for coordinates, way_tags in ways:
        if "building" in way_tags:
            polygon(ax, coordinates, "#d6cbc2", 0.80)
            label(ax, coordinates, way_tags.get("name"), color="#5f5650")

    named_roads = 0
    for coordinates, way_tags in ways:
        road_type = way_tags.get("highway")
        if road_type not in ROAD_STYLE or len(coordinates) < 2:
            continue
        width, color = ROAD_STYLE[road_type]
        xs, ys = zip(*coordinates)
        ax.plot(xs, ys, color="#b8ad9f", linewidth=width + 1.2, zorder=2, solid_capstyle="round")
        ax.plot(xs, ys, color=color, linewidth=width, zorder=3, solid_capstyle="round")
        if way_tags.get("name") and named_roads < 30:
            label(ax, coordinates, way_tags["name"], color="#574f42", fontsize=8)
            named_roads += 1

    node_points = [project(lat, lon, *origin) for lat, lon in nodes.values()]
    ax.scatter([point[0] for point in node_points], [point[1] for point in node_points],
               s=7, color="#35424c", alpha=0.75, zorder=4, label="OSM nodes")

    ax.set_title(f"OpenStreetMap: {map_path.name}")
    ax.set_aspect("equal", adjustable="box")
    ax.set_axis_off()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, facecolor=fig.get_facecolor())
    print(f"Nodes: {len(nodes)}, ways: {len(ways)}")
    print(f"Saved visualization: {output.resolve()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True, help="Standard OpenStreetMap .osm file")
    parser.add_argument("--output", type=Path, default=Path("raw_osm_map.png"), help="PNG output path")
    args = parser.parse_args()
    if not args.map.is_file():
        parser.error(f"map file does not exist: {args.map}")
    render(args.map, args.output)


if __name__ == "__main__":
    main()
