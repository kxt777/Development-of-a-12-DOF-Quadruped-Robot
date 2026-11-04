# dog_final

四足机器人整理版 `dog_final_20261030`。Windows 主目录：`D:\Project\dog_final`；NUC 部署目录：`/home/robot/dog_final`。

ROS 2 Humble 的常用节点统一从 **`dog_bringup/dog_system.launch.py`** 启动，根目录 `dog` 提供短命令。全部入口保持监控或预览模式，电机输出关闭。

## 最常用的命令

在 NUC 终端执行：

```bash
cd ~/dog_final
./dog status
./dog survey
```

`survey` 同时启动 MID-360、FAST-LIO、二维栅格投影、OAK 相机、三类 YOLO 和识别网页。识别画面：**http://192.168.30.176:8182**。

保存地图时，在第二个 NUC 终端执行：

```bash
cd ~/dog_final
./dog save-map
```

保存后回到运行终端按 `Ctrl+C` 结束。每次运行使用新的 `runs/时间_模式/`，三维地图为 `map.pcd`，二维地图为 `grid/map.yaml` 和 `grid/map.pgm`。已有文件不会被覆盖。

## 按功能启动

| 命令 | 启动内容 |
|---|---|
| `./dog status` | 只读查看节点、控制器与原有服务状态 |
| `./dog vision` | OAK RGB、对齐深度、YOLO、8182识别网页 |
| `./dog mapping` | MID-360、FAST-LIO、二维占据栅格 |
| `./dog survey` | 建图与识别一起启动，日常采集推荐入口 |
| `./dog save-map` | 保存本次建图的 PCD 和二维栅格 |
| `./dog monitor` | 独立 DM IMU 和 IK 监控；禁止电机写入 |
| `./dog plan` | 在保存地图上启动 Nav2 规划预览，起点为显式测试起点 |
| `./dog nav-monitor` | 实时定位、Nav2 和识别预览，运动输出固定关闭 |
| `./dog model` | 只显示 URDF 与 RViz，需要图形桌面 |
| `./dog doctor` | 检查配置、模型、依赖和设备占用 |
| `./dog args` | 查看统一 launch 参数，不启动节点 |
| `./dog test` | 运行本整理版检查，不接触硬件 |

上述启动模式互斥占用相同设备。程序遇到已有驱动或控制器时会明确报出进程并退出，不会自动终止旧服务。

NUC 整理时，旧的 `dog_ik_controller`、`dog_imu`、`dog_web` 服务已在运行，旧控制器的启动参数为 `allow_actuation=true`。这些服务及其启动配置保持原样。因此 `./dog monitor` 会拒绝与它们重复启动；`./dog status` 可直接查看现状，`survey` 不启动第二套步态控制器。

## 首次构建与参数

```bash
cd ~/dog_final
./dog build
./dog doctor
```

之后直接使用短命令，不必手动 source 多个工作区。`scripts/dog_env.sh` 自动按顺序加载 ROS、现有驱动依赖和本整理版工作区。

所有机器路径、设备端口、相机 Python、地图和识别端口集中在 **`config/site.json`**。部署配置中的运行目录、标定路径、轨迹缓存路径按本目录动态生成。

需要直接调用 ROS launch 时：

```bash
source scripts/dog_env.sh
ros2 launch dog_bringup dog_system.launch.py mode:=survey
```

指定其他地图或规划起点：

```bash
./dog plan map_yaml:=/absolute/path/map.yaml start_x:=0.6 start_y:=0.0
```

`nav-monitor` 的地图坐标对齐仍沿用既有流程，需要有效初始位姿；它不自动执行路线。保存的地图、传感器安装近似和点机器人配置保留其原有验证范围，不能据此认定全身可通行。

## 目录

```text
dog_final/
├─ dog                         常用命令入口
├─ config/                     设备配置、站姿和轨迹缓存
├─ ros2_ws/src/
│  ├─ dog_bringup/              统一 launch、启动占用检查、只读状态
│  ├─ dog_perception/           整理成 ROS 包的 YOLO 与识别网页
│  ├─ dog_navigation/           现有雷达、FAST-LIO、栅格、Nav2、四足桥接
│  ├─ dog_ik_walk/              实际部署使用的 IK、IMU航向、遥控源码
│  ├─ dm_imu/                   DM IMU 驱动
│  └─ dog_description/          URDF、mesh、RViz
├─ runtime/dog_tradition_real/  现有运动学、通信协议和标定运行库
├─ firmware/stm32h723/          STM32 H723 Keil 工程源码
├─ models/                     三类 YOLO 的 ONNX 和 PyTorch 权重
├─ maps/fire_scene/            火场地图与人工标注记录的副本
├─ scripts/                    构建、检查、保存地图工具
├─ docs/                       架构、来源清单、验证报告
├─ reference/                  上游依赖、训练和离线处理的追溯源码
└─ runs/                       每次运行的新输出；运行时生成
```

ROS 上游包和相机推理环境继续使用 NUC 已安装的版本；其路径在 `site.json` 中。`reference/third_party/` 保存 FAST-LIO、Livox 和已修改 Nav2 组件的源码副本。迁移到另一台 Ubuntu 时可参考 `docs/DEPENDENCIES.md` 重建这些依赖。

论文中的置信度主动观察闭环和时效终止模块属于后续算法实现；本次整理统一现有代码的目录、配置与启动方式。来源与具体边界见 `docs/ARCHITECTURE.md`、`docs/SOURCES.md`。
