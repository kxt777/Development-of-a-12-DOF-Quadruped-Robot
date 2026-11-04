# DOG 信号层：达妙 IMU 与电机 ROS 2 接入

目标是让 NUC 上的 ROS 2 Humble 接收真实电机反馈和 DM-IMU-L1 数据。
IMU 通过 USB 直连 NUC，电机通过 STM32 的 USB/CAN 桥接。
本阶段不启动 RL 推理、步态控制或自动电机使能。

## 来源与协议

协议与姿态转换参考工作区 `dm-imu-master.zip` 中的 ROS2-humble 例程，
以其中《DM-IMU-L1 六轴惯性测量单元使用说明书 V1.3 2026-09-22》
附录 E 为当前 USB 协议依据。该说明书对应固件 2.0.3.0。
保留官方例程的 MIT 许可，详见 LICENSE；压缩包及人形机器人参考项目未改动。

原例程只保留最后一帧且把任意类型当欧拉角，不能可靠组合多种输出。
本版按类型独立解析加速度、角速度、欧拉角和四元数，校验包含帧头的
CRC-16/CCITT-FALSE，处理串口分包、连续帧、噪声和重同步，并拒绝 NaN/Inf。
19 字节三轴帧与 23 字节四元数帧可混合读取。

| 输入 | ROS 输出 | 单位/顺序 |
| --- | --- | --- |
| USB 0x01 | `Imu.linear_acceleration` | m/s²，保留重力分量 |
| USB 0x02 | `Imu.angular_velocity` | rad/s，不再乘 π/180 |
| USB 0x03 | `Imu.orientation` | ZYX 欧拉角（度）转四元数 |
| USB 0x04 | `Imu.orientation` | 设备 WXYZ 转 ROS XYZW 并归一化 |
| STM32 0x11 | `/dog_motor_feedback` | JointState，位置 rad、速度 rad/s、力矩 N·m |
| STM32 0x11 | `/dog_motor_status` | 原始电机状态和两路温度、USB 接收状态 |

每条完整 IMU 消息必须包含更新过的加速度、陀螺仪和姿态。
默认各分量最大接收年龄 0.1 秒、接收时间差 0.03 秒。
缺失或超时则停止发布完整消息，通过 `/dog_imu/status` 提示。
各数据包没有设备时间戳或采样序号；这些检查按主机接收时间进行，
不等于硬件同步。同一串口读取批次使用同一个 ROS 接收时间。

协方差暂未知，保持 ROS 规定的全零，不虚构噪声参数。
`dog_imu_link` 表示传感器输出坐标系，当前不生成机身 TF、不移除重力、不归零航向。
设备安装方向、上位机内是否已经设置安装补偿，需要实测确认后统一。
六轴 IMU 的航向没有绝对参考，漂移不能当作通信故障。

## IMU 设备设置与 Windows 验证

用 USB 数据线连接 IMU，通过达妙上位机确认设备、固件版本、从机 ID。
选择 USB 测量输出，开启加速度、角速度，以及欧拉角或四元数。
建议第一轮把输出周期设置为 10 ms（100 Hz），确认数据后再决定提高频率。
若温控已开启，说明书规定需等待温度稳定后才输出测量数据。
本驱动不会改写输出配置、温控、校准或持久参数，也不发送旧版 USB 命令。

关闭占用 IMU 端口的达妙上位机和串口助手，然后在 PowerShell 执行：

```powershell
cd D:\Project\dog\dog_rl\dog_nuc\Dog_ROS2\src\dm_imu
python -m dm_imu.probe --list-ports
python -m dm_imu.probe --port COM5 --slave-id 1 --seconds 5
```

`COM5` 仅为示例，必须替换为实际 IMU 端口；不要用电机板的串口。
工具不依赖 ROS，依赖 pyserial。输出每种帧的频率、CRC 错误、完整样本数、
角速度、加速度和四元数；没有新鲜完整样本时返回非零退出码。

静置检查：加速度模长应接近当地重力加速度，角速度接近零。
依次绕壳体 X/Y/Z 正方向转动，核对轴对应和符号；这一项由现场人员完成。
如静止零偏明显，先固定设备，通过官方上位机做零偏校准。

## NUC 构建与统一启动

在已安装 ROS 2 Humble 的 NUC 上，把更新后的源码放到 `Dog_ROS2/src`。
不用复制参考项目的 build/install/log；当前工程已有的 install 也需要重新构建。

```bash
source /opt/ros/humble/setup.bash
cd <NUC上的Dog_ROS2目录>
sudo apt install python3-serial python3-colcon-common-extensions ros-humble-diagnostic-msgs
colcon build --packages-select dm_imu h723_bridge --symlink-install
source install/setup.bash
ls -l /dev/serial/by-id/
ros2 launch h723_bridge signals.launch.py motor_port:=/dev/ttyACM0 imu_port:=/dev/ttyACM1
```

默认端口沿用之前的 STM32 ACM0、IMU ACM1 约定，但插拔顺序可能改变编号。
现场确认两设备身份后，优先通过 launch 参数传入 `/dev/serial/by-id/...` 路径。
不在本阶段擅自写入 udev 规则。
`imu_slave_id` 默认 1，`imu_frame_id` 默认 `dog_imu_link`，均可通过 launch 覆盖。

统一入口启动 `dog_h723_bridge` 和 `dm_imu` 两个独立节点，共用 NUC 的 ROS 时钟。
默认 `monitor_only:=true`，电机命令话题会被忽略，且不会自动使能电机。
单独原有 `bridge.launch.py` 的控制行为保持原样。
退出只关闭串口；该只读入口不提供急停、自动失能或硬件看门狗功能。

另一个终端 source 相同环境后验证：

```bash
ros2 topic hz /dog_motor_feedback
ros2 topic echo /dog_motor_status --once
ros2 topic hz /dog_imu/data
ros2 topic echo /dog_imu/data --once --qos-reliability best_effort
ros2 topic echo /dog_imu/status --once
```

电机 USB 反馈预期约 50 Hz；IMU 发布频率由设备输出配置和完整三类数据决定。
STM32 固件的 valid 字节固定为 1，尚无逐电机 CAN 接收时间戳/在线标志。
因此电机诊断项保持 WARN，明确展示“缓存快照，CAN 新鲜度未知”；
USB 收包正常不能证明 12 台电机都在线。
Windows 已实测 8 号 MIT 移动；其余电机逐台验收不在本次 IMU 接入中代替完成。

## 验证与阶段边界

纯协议测试不需要 ROS：

```powershell
cd D:\Project\dog\dog_rl\dog_nuc\Dog_ROS2
$env:PYTHONPATH = "$PWD\src\dm_imu;$PWD\src\h723_bridge"
python -m unittest discover -s src/dm_imu/test -v
python -m unittest discover -s src/h723_bridge/test -v
```

代码离线测试通过不代表硬件、ROS 构建和节点发布已经实测通过。
本阶段验收需要记录：两路实际串口、IMU 固件/从机 ID/输出周期、静止数据、
三个轴的转动符号、ROS 话题频率及断开 IMU 后的 WARN/ERROR 和重连恢复。
IMU 与机身坐标关系、关节零位/方向及训练观察量映射仍需后续确认。

ROS 数据语义依据：
[Humble Imu 消息定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg)、
[REP 145](https://github.com/ros-infrastructure/rep/blob/master/rep-0145.rst)。
