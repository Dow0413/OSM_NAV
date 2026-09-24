# map_viz

Render a Lanelet2 `.osm` map to a PNG. With a local-metre start and goal, the package matches both locations to a drivable lanelet and overlays the Dijkstra shortest vehicle route.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run map_viz osm_map_viz.py --output map.png
ros2 run map_viz osm_map_viz.py --start 1818.04 291.77 --goal 1743.10 370.25 --output route.png --show
```

For another map, pass `--map /path/to/map.osm` and use an origin matching the map coordinates, for example `--origin 49.0 8.4`.
