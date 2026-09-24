# Galileo OSM navigation: simulated traffic lights

Run with `mode: 1` in `config/galileo_osm_nav.yaml`, then build and launch:

```bash
colcon build --packages-select galileo_osm_nav --symlink-install
source install/setup.bash
ros2 launch galileo_osm_nav galileo_osm_nav.launch.py
```

Open `http://127.0.0.1:8090/`. Expand **模拟交通信号灯** below **机器狗基础通行规则**. Each switch sets that light to `1` (red) or `0` (green). All lights start red when the server starts; state is held in memory. `mode: 0` shows the switches but does not command the robot.

In `mode: 1`, **鼠标当前位置** shows the cursor's latitude/longitude and the corresponding `/Odometry`-frame x/y (metres), computed with the same `paired.yaml` inverse transform used to publish `/global_path`. These x/y values describe the map point under the cursor, not the robot's latest odometry pose.

The route planner associates a pedestrian way (`footway`, `path`, `pedestrian`, or `steps`) with a signal when the way contains an OSM `highway=traffic_signals` node, or when the way has `crossing=traffic_signals`. Only signals associated with a pedestrian crossing appear in the web panel and on the map; unrelated signals do not stop the robot. If multiple signal nodes appear on one way, each segment uses the nearest signal node.

In mode 1, the bridge projects `/Odometry` onto the active `/global_path`. If the robot is approaching a controlled pedestrian segment, its signal is red, and the entry is within `signal_stop_distance_m` (default 2 m), the bridge repeatedly publishes `2` to `stop_nav` (default `/stop`) and zero `Twist` to `stop_override_topic` (default `/cmd_vel/final_align`). The latter is the priority input of the downstream `cmd_vel_mux`; it keeps the final command at zero even if another navigation node publishes `/stop=0`. Once the light turns green, the bridge publishes `/stop=0`, stops the zero override, and the mux returns to the existing path follower after its configured override timeout (currently 0.35 s). The path itself is retained. A light switching red after the robot has entered a crossing does not stop it mid-crossing.

The signal controls require the downstream stack's `cmd_vel_mux.override_topic` to match `stop_override_topic`. This is a demo control layer; set the stop distance for the robot's actual speed and braking distance before using it outside simulation.

## PCD and OSM building alignment

Set `pcd_overlay: true` and `pcd_map_dir` in `config/galileo_osm_nav.yaml`. The directory needs `manifest.yaml` and a binary float32 `x y z intensity` PCD named by `global_map_file`. The server samples the PCD and makes transparent, north-up PNG overlays; it does not send the 76 MB raw cloud to the browser. The layer starts hidden.

In the web sidebar, expand **点云与 OSM 建筑对齐** and enable the PCD projection. Use **缩放到点云范围**. Cyan pixels show PCD points with `z >= 3 m`; orange lines show OSM building polygons. Switch to **全部点** if you need the ground/road context. Adjust opacity to compare the cyan geometry with the orange footprint and, when `use_leaflet: true`, with the satellite image.

The manifest contains `enu_origin_lla` and `t_align` but does not say whether the saved PCD has already been transformed into ENU. The selector therefore offers raw XY as ENU, applying `t_align`, and applying its inverse. Compare recognizable corners in all three modes. This visual check does not recalibrate the navigation `paired.yaml` transform. For a numeric alignment estimate, identify at least three matching surveyed corners in the PCD and OSM/GNSS data.
