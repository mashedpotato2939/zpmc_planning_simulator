# 厦门高精地图

本目录同时提供两种矢量地图。规划启动脚本默认加载厦门的 Lanelet2 OSM 地图，也可以切换为 ZPMC 瓦片地图。

| 文件 | 用途 |
| --- | --- |
| `xiamen-prod-galactic.osm` | Lanelet2 OSM 地图，当前默认地图 |
| `map_header.bin`、`tile_*.bin` | ZPMC 瓦片地图 |

本目录目前没有 `global.pcd`。如需在 RViz 中显示点云地图，须自行准备与矢量地图坐标一致的 PCD 文件。

## 启动和切换地图格式

在工作区根目录执行：

```bash
./launch_with_fms_real_vehicle.sh
```

上述命令默认使用厦门 OSM 地图。要使用本目录中的瓦片地图：

```bash
MAP_FORMAT=tiles ./launch_with_fms_real_vehicle.sh
```

`MAP_FORMAT` 只接受 `osm` 和 `tiles`，每次启动只加载其中一种矢量地图。切换格式只需修改启动命令中的环境变量，不需要改规划节点源码。

## 设置地图路径

以下环境变量由 `launch_with_fms_real_vehicle.sh` 传给 `igv_planning.launch.xml`：

| 环境变量 | 默认值 | 用途 |
| --- | --- | --- |
| `MAP_FORMAT` | `osm` | 矢量地图格式：`osm` 或 `tiles` |
| `MAP_PATH` | 工作区的 `src/map/xiamen_tiles_v1` | OSM 和可选点云地图所在目录 |
| `LANELET2_MAP_FILE` | `xiamen-prod-galactic.osm` | OSM 文件名；实际路径为 `MAP_PATH/LANELET2_MAP_FILE` |
| `ZPMC_TILE_DIRECTORY` | 工作区的 `src/map/xiamen_tiles_v1` | 瓦片地图目录，须包含 `map_header.bin` 和对应的 `tile_*.bin` |
| `LAUNCH_POINTCLOUD_MAP` | `false` | 是否同时加载点云地图 |
| `POINTCLOUD_MAP_FILE` | `global.pcd` | PCD 文件名；实际路径为 `MAP_PATH/POINTCLOUD_MAP_FILE` |

使用其他 OSM 地图时，分别指定目录和文件名：

```bash
MAP_FORMAT=osm \
MAP_PATH=/data/maps/my_map \
LANELET2_MAP_FILE=my_map.osm \
./launch_with_fms_real_vehicle.sh
```

使用其他瓦片地图时，指定瓦片目录：

```bash
MAP_FORMAT=tiles \
ZPMC_TILE_DIRECTORY=/data/maps/my_tiles \
./launch_with_fms_real_vehicle.sh
```

如果地图目录中已有 `global.pcd`，可同时加载点云地图供 RViz 显示：

```bash
MAP_FORMAT=osm \
MAP_PATH=/data/maps/my_map \
LAUNCH_POINTCLOUD_MAP=true \
POINTCLOUD_MAP_FILE=global.pcd \
./launch_with_fms_real_vehicle.sh
```

当前启动配置从 `MAP_PATH` 查找 PCD，因此 OSM 与 PCD 应放在同一目录。瓦片模式也可开启点云加载，此时 `ZPMC_TILE_DIRECTORY` 指向瓦片目录，`MAP_PATH` 指向 PCD 所在目录。没有 PCD 时保持 `LAUNCH_POINTCLOUD_MAP=false`。

## 修改默认地图

- 使用启动脚本时，默认值定义在工作区根目录的 `launch_with_fms_real_vehicle.sh`，直接设置上述环境变量即可临时覆盖。
- 直接执行 `ros2 launch igv_launch igv_planning.launch.xml` 时，对应的启动参数为 `map_format`、`map_path`、`lanelet2_map_file`、`zpmc_tile_directory`、`launch_pointcloud_map` 和 `pointcloud_map_file`；其默认值定义在 `src/launch/igv_launch/launch/igv_planning.launch.xml`。
- 如需永久更换默认格式或文件名，请同步修改上述脚本和 launch 文件的默认值。若新增地图文件需要随 `xiamen_tiles_v1` 包安装，还要将文件加入本目录 `CMakeLists.txt` 的 `install(FILES ...)` 列表，并重新构建相关包、刷新环境：

```bash
colcon build --packages-select xiamen_tiles_v1 igv_launch --symlink-install
source install/setup.bash
```

## `hdmap` 依赖与源码

- **OSM 模式**由 `src/planning_simulator_components/map/map_loader` 中的 `lanelet2_map_loader` 读取，使用 Lanelet2 和 `autoware_lanelet2_extension`，不调用 `hdmap`。
- **瓦片模式**由 `src/utils/zpmc_map_loader` 读取，其 `CMakeLists.txt` 包含 `find_package(hdmap REQUIRED)`，因此编译瓦片加载器需要 `hdmap`。
- 当前工作区的 `install/hdmap` 里有已安装的头文件、静态库和转换工具，但 `src` 下没有 `hdmap` 源码。若重新编译整个工作区并包含 `zpmc_map_loader`，须确保 CMake 能找到这份 `hdmap` 安装包；只运行已构建好的 OSM 加载链路不需要调用它。

## 瓦片地图来源

旧版记录显示，这组瓦片由 `map/xiamen-prod.osm` 使用 `hdmap` 提交 `141ca4300eeb87d7d1283dd7b2d1332dca4052bc` 生成；转换参数为端点阈值 `2.0`、瓦片尺寸 `300`、采样分辨率 `0.15`。当时记录的源 OSM SHA-256 为 `a658d326fb99ca223d1079320696adc00475ffd79f43cfd78dba742944ca5d97`。

旧版记录中的生成脚本为 `agv_maps/scripts/generate_xiamen_zpmc_tiles.sh`。当前工作区没有该脚本和上述 `map/xiamen-prod.osm` 源文件，因此不能直接在本仓库中按旧命令重新生成瓦片。
