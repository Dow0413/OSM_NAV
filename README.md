1. 修改 `src/osm_nav/config/osm_nav.yaml` 中的地图与运行参数。
2. 工作区只需包含 `src/osm_nav`；完整 Lanelet2 源码已内嵌在其 `src/lanelet2/` 下。
3. `colcon build --packages-select osm_nav` 编译。
4. `ros2 launch osm_nav osm_nav.launch.py` 启动。
5. 打开 `http://localhost:8090/`。
