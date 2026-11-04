# 验证记录 — 2026-10-30

目标 NUC：`robot@192.168.30.176`。整理版已部署至 `/home/robot/dog_final`。

| 检查 | 结果 |
|---|---|
| 部署文件 SHA256 | 首次部署的541个清单文件逐一匹配 |
| ROS构建 | dm_imu、dog_ik_walk、dog_navigation、dog_description、dog_perception、dog_bringup，6包全部成功 |
| 整理版配置与启动约束 | 本地Windows、NUC均8项通过 |
| 既有步态/航向/遥控测试 | 44项通过，使用内存假通信，不连接真实电机 |
| 既有导航单元测试 | ROS_DOMAIN_ID=49、localhost隔离，21项通过 |
| 统一launch参数 | `./dog args`通过，6个参数可读，无节点启动 |
| 模块加载路径 | 步态、导航、bringup、视觉均从dog_final工作区加载 |
| 原控制器冲突 | `./dog monitor`检测到原PID1004/1005并退出，未终止旧进程 |
| 电机只读约束 | 实例化监控节点；Serial构造被测试替身拦截，未被调用，电机写入次数0 |
| Nav2实际规划 | 新工作区在保存地图上规划成功，起点(0.6,0.0)、终点(1.45,0.71)，285个路径采样均为空闲，端点误差0 |
| 视觉包CLI与ROS传输 | 使用独立域49和训练图片夹具；收到Detection2DArray及HTTP状态，模型错误0；未使用实体相机或深度数据 |
| 原源码与冻结基线 | 整理前后摘要一致，见protected_originals_sha256.json |
| 原NUC用户服务 | controller/IMU/web仍active，主PID分别1004/1005/1006，与整理前相同 |

规划验证是保存地图上的点机器人规划，不是真机路线或避障试验。视觉夹具仅验证打包后的启动参数和传输，不是新增识别准确率或烟雾实测。此次没有启动实体相机、雷达采集，没有电机使能、运动或服务迁移。

只读状态查询时，旧控制器报告 `allow_actuation=true`、`enabled=false`、`mode=null`。本整理版统一入口始终关闭运动输出；不据此改动原服务。

原始日志、规划结果和夹具测试结果在 `verification_20261030/`。构建日志为 `build.log`，摘要为 `smoke_summary.json` 和 `vision_transport_summary.json`。
