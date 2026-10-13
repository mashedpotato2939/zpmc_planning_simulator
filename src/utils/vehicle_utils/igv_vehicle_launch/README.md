# igv_vehicle_model

振华车辆模型autoware可视化仓库

## 1. 启动方式

需要放入autoware/src/vehicle：

```bash
cd autoware/src/vehicle
git clone git@gitee.com:csc105/igv-vehicle-launch.git
```

编译（只需要编译这两个包）

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_STANDARD=17 --parallel-workers 24 --packages-up-to igv_vehicle_launch igv_vehicle_description
```

在启动时修改vehicle_model:=igv_vehicle

```bash
ros2 launch autoware_launch autoware_carla.launch.xml vehicle_model:=igv_vehicle sensor_model:=carla_sensor_kit map_path:=/home/cjh/Project/map/custom_junctions_with_boundary
```
