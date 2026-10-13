# 直角弯轨迹生成器

## 概述

本项目实现了一个基于**回旋线-圆弧-回旋线 (Clothoid-Arc-Clothoid)** 的直角弯轨迹生成算法，用于 AGV/自动驾驶车辆在狭窄空间内完成 90 度转弯的路径规划。

第一步根据起点、终点和半径生成直角弯曲线，这一步计算出的航向角是曲线的切线方向；第二步根据上一步直角弯沿弧长重新分配航向角，保证总角度为90度，这是AGV实际运行所跟踪的航向角；第三步，将第一步航向角和第二步航向角沿弧长作差，得到的是侧滑角。侧滑角*纵向速度=横向速度。

map_traj_server.cpp
├── 积分函数
│   ├── adaptive_simpson()    # 自适应 Simpson 积分
│   ├── fresnelc()            # 菲涅尔积分 C(x)
│   └── fresnels()            # 菲涅尔积分 S(x)
├── 轨迹计算函数
│   ├── clothoid_point()      # 回旋线单点计算
│   ├── clothoid_local_offset() # 回旋线局部偏移
│   ├── compute_endpoint_x()  # 计算终点 x 坐标
│   ├── compute_endpoint_y()  # 计算终点 y 坐标
│   └── solve_clothoid_arc()  # 求解轨迹参数
├── 采样函数
│   ├── sampling_points_xy()  # 生成轨迹点坐标
│   └── sampling_points_yaw() # 生成航向角和曲率
└── 轨迹生成函数
    └── traj_creator_turn_optimize()    # 参数配置与轨迹生成


# 斜行轨迹生成器
map_traj_server.cpp
├── 轨迹计算函数
│   └── crabwalking_traj_generate()  # 求解轨迹参数
├── 采样函数
│   └── calcPosy_Vy()                 # 生成横向位置和速度
└── 轨迹生成函数
    └── traj_creator_crabwalking()    # 参数配置与轨迹生成