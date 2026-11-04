# 整理版架构

统一入口：`./dog <模式>` → `dog_bringup/launch/dog_system.launch.py`。原功能 launch 保留为内部组件，用户不再需要分别启动它们。项目节点名称均使用 `dog_` 前缀。

| 模式 | 核心节点 | 主要输出 |
|---|---|---|
| monitor | dog_imu、dog_ik_walk_imu | /dog_imu/data、/dog_ik_walk_imu/status |
| vision | dog_oak及其容器、dog_custom_yolo | /dog_vision/dog_oak/*、/dog_vision/dog_custom_yolo/* |
| mapping | dog_lidar、dog_fastlio、dog_octomap | /dog_lidar/*、/dog_mapping/*、/dog_grid/map |
| survey | mapping + vision | 几何地图和带分数识别，分别输出 |
| plan | dog_planning_start_tf、dog_map_server、dog_planner_server、dog_planning_lifecycle_manager | /dog_nav/map、/dog_nav/compute_path_to_pose |
| nav-monitor | 实时雷达与定位、Nav2服务、dog_nav_velocity_bridge、视觉 | /dog_nav/*；velocity_preview只供预览 |
| model | dog_robot_state_publisher、dog_rviz2 | URDF可视化 |

`survey` 沿用各前端原始坐标系和时间戳，不额外伪造相机到地图的标定关系，不把离线试投影升级为在线标定结果。识别框上的分数为固定 YOLO 模型单帧输出。

## 启动控制

1. 读取 `config/site.json`，确定模式所需资源。
2. 取得本整理版的资源锁，防止两个新 launch 竞争同一设备；Nav2 实时和离线预览共享锁。
3. 检查旧工作区及其他程序的相机、雷达、IMU、控制器占用。发现冲突直接退出，旧进程保持运行。
4. 创建全新的运行目录，生成当前机器路径的配置。
5. 启动所选 ROS 组件。统一入口固定传入 `allow_actuation=false` 和 `allow_motor_output=false`，不提供使能或执行路线的快捷命令。

所有源码中的既有运动接口为追溯和后续测试保留；使用统一入口时它们不获电机输出权限。`monitor` 不打开电机串口，不能代替实际运行的旧控制器提供电机反馈；查看旧控制器使用 `./dog status`。

## 原版保存

- 原 Windows 项目 `D:/Project/dog` 保留。
- 原 NUC 工作区、systemd 用户服务、活动控制配置保留。
- 冻结导航基线 `dog_nav_v6_three30_20261024`、其源码/地图/路线/实验证据保留。
- 当前目录是新的整理版本，不是对冻结版的覆盖或重新命名。

## 现有能力边界

保留已实现的传感器接入、FAST-LIO、二维占据栅格、三类 YOLO、Nav2 和离散四足执行桥。论文新提出的多帧置信度监督、观察位置重规划和时效异常终止尚需单独工程实现与闭环实验。安装外参与机身足迹沿用原有记录；完整机身避障与真实火场适用性需要进一步验证。
