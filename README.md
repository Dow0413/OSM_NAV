# OSM_NAV

1. 修改 `src/osm_nav/config/osm_nav.yaml` 中的地图与运行参数。
2. 工作区只需包含 `src/osm_nav`；完整 Lanelet2 源码已内嵌在其 `src/lanelet2/` 下。
3. `colcon build --packages-select osm_nav` 编译。
4. `ros2 launch osm_nav osm_nav.launch.py` 启动。
5. 打开 `http://localhost:8090/`。

- osm_nav.yaml中需要选择当前使用的类别（人、车、狗），每一种类别可以在topo_setting.yaml中设置不同的拓扑要求，每一种类别也都归属于一个大类，在traffic_info.yaml设置信息灯约束、转向限制等参数

- 显示栏可以调整显示设置

- 导航栏可以在设置起点终点后进行规划，当mode=1时，起点由odometry_topic获取

- 模拟交通栏可以模拟交通信号灯的实时情况，后续和视觉结合

- 规则栏可以看当前使用的交通规则，暂时没用

- 类别栏可以看各个线路/道路类别的分布情况

- 点云显示栏可以用RTK+雷达融合的PCD地图叠加到卫星地图中显示。PCD/OSM三维校图可以使用卫星地图+PCD地图对osm地图进行微调，也可以生成SLAM定位与经纬度定位的配对文件

- 规则设计栏可以给每条线路设定信息灯约束、转向限制等

- OSM编辑栏中可以在编辑器中对osm地图进行编辑，添加点、线、区域等设置