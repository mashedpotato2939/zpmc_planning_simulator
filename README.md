# ZPMC 规划、Gazebo、感知联合仿真启动指南

本文记录当前厦门码头场景的联合启动流程。使用四个终端分别运行 Gazebo、规划测试组件、规划主链路和感知链路。Gazebo 使用 ROS 2 Humble 容器；规划运行在宿主机的 ROS 2 Galactic 环境；感知使用 ROS 2 Galactic 容器。

## 目录与前提

| 模块 | 本机目录 | 运行位置 |
| --- | --- | --- |
| Gazebo | `/home/mash/gazebo_simulator/igv_gazebo/igv` | `gazebo-test` 容器，挂载到 `/igv` |
| 规划 | `/home/mash/Zpmc_planning` | 宿主机 |
| 感知 | `/home/mash/gazebo_simulator/others/perception/autoware.universe.galactic/autoware.universe` | `zpmc-perception` 容器，工作空间位于 `/workspace/autoware.universe` |

先确认两个容器在运行：

```bash
docker ps --format '{{.Names}} {{.Status}}' | grep -E '^(gazebo-test|zpmc-perception) '
```

如果容器已经创建但处于停止状态，先运行 `docker start gazebo-test zpmc-perception`。如果容器不存在，需要先按各项目的容器部署说明创建，并保持上表中的目录挂载关系。

规划工作空间需要已有 `install/setup.bash`。Gazebo 启动脚本默认先编译 `agv_gazebo`；感知工作空间需要已有 `/workspace/autoware.universe/install/setup.bash`。首次搭建容器和编译感知工作空间时，可参考感知项目的 `README.md`。以下命令使用现有容器，不创建新容器。

三个模块的 ROS 通信域均使用 `ROS_DOMAIN_ID=0`。Gazebo 是联合仿真中唯一的 `/clock` 发布者；规划和感知节点使用仿真时间。Gazebo 启动文件会为本次运行生成独立的 Gazebo Transport 分区，避免连接到上次启动留下的服务地址。

## 启动顺序

### 终端 1：Gazebo

在宿主机运行：

```bash
cd /home/mash/gazebo_simulator/igv_gazebo/igv
GAZEBO_INTERNAL_DOMAIN_ID=0 \
GAZEBO_ENABLE_REMOTE_EXPORT=false \
GAZEBO_ENABLE_LEGACY_SENSOR_ALIASES=true \
GAZEBO_ENABLE_SECONDARY_LIDARS=false \
GAZEBO_INITIAL_Y=28.0013 \
./launch_agv_gazebo.sh
```

脚本进入 `gazebo-test` 容器，默认加载 `xiamen_empty.sdf`、`performance` 传感器配置和 Gazebo GUI。当前感知拼接配置订阅 `/rslidar_points_2`、`/rslidar_points_4`、`/hesai_left_front`、`/hesai_right_rear`，因此这里必须启用 `GAZEBO_ENABLE_LEGACY_SENSOR_ALIASES=true`。上扫雷达暂未用于这四路输入；需要六路雷达时再设置 `GAZEBO_ENABLE_SECONDARY_LIDARS=true` 并核对感知输入配置。

等 Gazebo 场景加载完成，且 `/clock` 开始递增后，再启动其他模块。`GAZEBO_INITIAL_Y=28.0013` 对应当前已验证的启动配置；规划测试组件稍后会通过 `/localization/kinematic_state` 更新 Gazebo 中的车辆位姿。

### 终端 2：规划测试组件与控制链

在宿主机运行：

```bash
cd /home/mash/Zpmc_planning
export ROS_DOMAIN_ID=0
LAUNCH_SIMULATION_CLOCK=false \
LAUNCH_DUMMY_PERCEPTION=false \
LAUNCH_SIMPLE_OBJECT_PREDICTOR=false \
./launch_fms_test_components.sh
```

该脚本启动规划用的简单车辆模拟器和控制链，并发布 `/localization/kinematic_state`。这里关闭独立仿真时钟，防止与 Gazebo 同时发布 `/clock`；也关闭假感知和备用预测器，避免与真实感知链路重复发布。

### 终端 3：规划主链路

在宿主机运行：

```bash
cd /home/mash/Zpmc_planning
export ROS_DOMAIN_ID=0
./launch_with_fms_real_vehicle.sh
```

脚本启动地图、FMS 路径规划、`vehicle_agent`、感知消息转接、速度规划及 RViz。当前默认 `VEHICLE_AGENT_ADAPTERS=ros_injector`，用于从 RViz 注入测试任务。待所有模块就绪后，在 RViz 的 FMS 面板将 **Publish via** 设为 **`vehicle_agent`**，再下发路线。

### 终端 4：感知

在宿主机另开终端，进入现有感知容器：

```bash
docker exec -it zpmc-perception bash
```

在容器内运行：

```bash
source /opt/ros/galactic/setup.bash
source /workspace/autoware.universe/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch autoware_launch sim_perception.launch.xml rviz:=false
```

`sim_perception.launch.xml` 默认使用 Gazebo 的 `/clock` 和仿真感知配置，不再发布一套传感器外参 TF。这里关闭感知容器内的 RViz，使用规划终端的 RViz 查看结果。不要用实车入口 `khy_perception.launch.xml` 或 `zpmc_perception.launch.xml` 代替该仿真入口。

## 启动后检查

在宿主机新终端执行：

```bash
source /opt/ros/galactic/setup.bash
source /home/mash/Zpmc_planning/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 topic info /clock -v
ros2 topic hz /clock
```

`/clock` 应持续递增，且只有一个发布者。按 `Ctrl+C` 结束 `ros2 topic hz`，然后核对关键话题：

```bash
ros2 topic list | grep -E '^(/localization/kinematic_state|/rslidar_points_2|/rslidar_points_4|/hesai_left_front|/hesai_right_rear|/perception/centerpoint/objects|/perception/ground_consistency_object_detector/occupancy_grid|/fms/route_window)$'
```

原始雷达话题出现后，感知链路应逐步输出 `/sensing/lidar/concatenated/pointcloud`、`/perception/centerpoint/objects` 和 `/perception/ground_consistency_object_detector/occupancy_grid`。任务下发后，可检查 `/fms/route_window` 与 `/planning/scenario_planning/trajectory`。

如果点云或障碍物没有更新，先检查 `/clock`、四路雷达话题和 `ROS_DOMAIN_ID`。如果 TF 提示时间回退，检查是否还有其他 `/clock` 发布者；Gazebo 重启后，最好按下面的顺序重启依赖它的模块。若 Gazebo GUI 缺少岸桥或道路，检查启动日志中是否仍有 `Unable to find file with URI`，并确认使用的是当前的 Gazebo 启动文件。

## 停止与重启

依次在感知、规划主链路、规划测试组件、Gazebo 的终端按 `Ctrl+C`。Gazebo 最后停止，可以让其他节点在退出时仍获得 `/clock`。重新启动 Gazebo 会重置仿真时间；若已运行的规划或感知节点出现 TF 时间回退，按上述顺序停止后重新启动整套链路。

## 对应入口

- Gazebo：`/home/mash/gazebo_simulator/igv_gazebo/igv/launch_agv_gazebo.sh`
- 规划测试组件：`launch_fms_test_components.sh`
- 规划主链路：`launch_with_fms_real_vehicle.sh`
- 仿真感知：`sim_perception.launch.xml`
