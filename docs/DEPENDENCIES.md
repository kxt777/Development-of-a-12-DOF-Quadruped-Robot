# 依赖与构建

目标系统：Ubuntu 22.04、ROS 2 Humble。Windows 保存与编辑源码，ROS 节点在 NUC 运行。

## 当前 NUC

`./dog build` 只构建六个项目包。已有第三方依赖通过 `config/site.json` 指定的 underlay 加载，本项目包作为最后一层 overlay 使用。

视觉使用现有的独立 `--system-site-packages` Python 环境，保持系统 ROS 的 numpy/cv_bridge 兼容关系。若本目录建立了 `.venv/bin/python`，统一 launch 优先使用它；否则使用 `site.json` 的 `vision_python`。原模型和推理参数保留。

## 迁移到新机器

1. 安装 ROS 2 Humble、colcon、系统 numpy/scipy/pyserial/OpenCV，以及项目 package.xml 声明的 ROS 依赖。
2. 安装 Livox SDK2、PCL、Eigen 和 FAST-LIO、Livox 源码要求的其他开发库。相机使用 `depthai_ros_driver_v3`；确认其 OAK 固件与驱动匹配。
3. `reference/third_party/` 为源码副本，含项目使用过的 FAST-LIO帧名和 Nav2 dog costmap 名称等修改。需要独立依赖工作区时执行 `./dog build-drivers`，再执行 `./dog build`。此命令不安装系统包、不改原工作区、不触发传感器或电机。
4. 创建独立视觉 Python 环境，安装与部署记录兼容的 ONNX Runtime；更新 `site.json` 的依赖、Python和设备路径。
5. 运行 `./dog doctor`、`./dog test`、`./dog args`，再按需要启动传感器。

记录中的 ONNX SHA256：`dcd1544872485062ee3bd7619914442992dc4c9ee89db4f55c325791b5f28cb9`。

`./dog build-drivers` 是保留的迁移工具，当前整理任务使用既有 NUC 驱动依赖，不以重新编译驱动作为原运行版本的替换。
