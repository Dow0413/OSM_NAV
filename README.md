1. 修改congfig文件中的osm路径（要求是绝对路径）
2. 按照下面的格式放
    src
        galileo_osm_nav
        map_viz
        Lanelet2
3. colcon build编译，一共13个包
4. ros2 launch galileo_osm_nav galileo_osm_nav.launch.py
5. 打开localhost:port