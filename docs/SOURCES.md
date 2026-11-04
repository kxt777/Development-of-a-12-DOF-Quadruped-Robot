# 来源与整理说明

整理日期：2026-10-30（Asia/Shanghai）。以 NUC 的实际运行路径和本地现有工程为来源，不按文件夹名字推定最新版本。

| 整理后位置 | 来源 |
|---|---|
| ros2_ws/src/dog_navigation | NUC /home/robot/dog_navigation/ws/src/dog_navigation |
| ros2_ws/src/dog_ik_walk | NUC versions/dog_nav_input_v041_20261021/source/dog_ik_walk；实际控制服务通过 PYTHONPATH 加载此版本 |
| ros2_ws/src/dm_imu | NUC dog_ik_walk_imu_ws_20261007_v031/src/dm_imu |
| runtime | NUC dog_ik_walk_imu_ws_20261007_v031/runtime |
| config/source | 当前控制器部署配置、实际使用的站姿与轨迹缓存副本 |
| ros2_ws/src/dog_description | 本地 dog_rl/dog_nuc/Dog_ROS2/src/dog_description |
| ros2_ws/src/dog_perception | NUC dog_vision_custom_20261026/scripts；只调整包内导入和ROS命令行参数兼容 |
| firmware/stm32h723 | 本地 dog_rl/dog_stm32/Keil_stm32h723；保留源码和工程，排除编译缓存 |
| models | 已部署 ONNX 与其本地 PyTorch 原权重 |
| reference/training | 本地 dog_vision_training/v1_20261026 的训练、标注脚本及版本记录 |
| reference/offline_processing | demov3 离线地图、关键帧、轨迹审计、CAD初始外参处理源码 |
| reference/third_party | NUC 实际安装来源中的 FAST-LIO、Livox、Nav2 planner/controller；保留其许可证 |

来源文件的 SHA256 和路径见 `nuc_source_manifest.json`、`nuc_active_source_manifest.json`、`local_source_manifest.json`。后一次 NUC 活动源码清单覆盖同名的早期步态包；早期清单保留追溯意义。

`reference/` 中的历史独立启动脚本、训练与离线处理脚本保留当时路径，供方法追溯；正式日常入口为根目录 `dog`。旧版资料、实验视频、全部历史部署归档不重复复制，仍在 `D:/Project/dog`。

整理修改限于新增统一 bringup、构建/检查/保存工具、运行时路径迁移，以及将既有视觉脚本打包成 ROS 包。运动学、USB/CAN协议、电机映射、步态参数和原冻结工作部署不修改。
