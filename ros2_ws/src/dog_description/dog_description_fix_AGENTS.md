# AGENTS.md --- dog_description ROS2 纠错与整理任务

## 项目背景

你正在处理一个 ROS2 Humble 四足机器人项目中的 `dog_description` 包。

当前目标：

修复并规范化机器人描述包，使其能够稳定完成：

    URDF
     ↓
    robot_state_publisher
     ↓
    joint_state_publisher_gui
     ↓
    RViz2

后续还需要作为真实机器人控制入口：

    ROS2
     ↓
    h723_bridge
     ↓
    USB CDC
     ↓
    STM32H723
     ↓
    FDCAN
     ↓
    DM4340 ×12

因此请不要破坏已有 URDF、mesh、joint 命名和后续接口。

------------------------------------------------------------------------

# 任务目标

## 1. 修复 display.launch.py

重点检查：

    dog_description/launch/display.launch.py

要求：

-   使用 ROS2 Humble 推荐 Python launch 写法。
-   正确加载 `dog.urdf`。
-   正确传递 `robot_description` 参数。
-   避免 YAML parsing error：

```{=html}
<!-- -->
```
    Unable to parse the value of parameter robot_description as yaml

推荐结构：

``` python
ParameterValue(
    Command(["cat", urdf_file]),
    value_type=str
)
```

注意：

不要使用：

``` python
Command(["cat ", urdf_file])
```

避免字符串空格导致解析问题。

------------------------------------------------------------------------

# 2. 检查 launch 文件重复问题

检查：

    launch/
    ├── display.launch
    └── display.launch.py

要求：

-   明确 ROS2 Humble 使用 `.launch.py`
-   如果 `.launch` 是旧文件或者 ROS1 遗留文件：
    -   不删除
    -   添加说明
    -   避免两个文件行为不一致

最终推荐：

    display.launch.py

作为主要入口。

------------------------------------------------------------------------

# 3. 保证 URDF 加载路径正确

检查：

    urdf/dog.urdf

不要硬编码绝对路径。

必须使用：

    FindPackageShare("dog_description")

保证：

源码空间：

    src/dog_description

和安装空间：

    install/dog_description/share/dog_description

均可工作。

------------------------------------------------------------------------

# 4. 检查 package.xml 和 setup 配置

确认：

launch 文件能够被 colcon 正确安装。

执行：

    colcon build

后：

    install/dog_description/share/dog_description/launch/

必须存在：

    display.launch.py

------------------------------------------------------------------------

# 5. 保留现有机器人结构

禁止修改：

## robot名称

    dog

## base link

    base_link

## 关节名称

必须保持：

    fl_1
    fl_2
    fl_3

    fr_1
    fr_2
    fr_3

    bl_1
    bl_2
    bl_3

    br_1
    br_2
    br_3

这些名称后续用于：

ROS2 joint_states

以及：

电机ID映射。

------------------------------------------------------------------------

# 6. 验证标准

修复完成后必须验证：

## 编译

    colcon build

成功。

------------------------------------------------------------------------

## 启动

    source install/setup.bash

    ros2 launch dog_description display.launch.py

成功打开：

-   RViz2
-   joint_state_publisher_gui

------------------------------------------------------------------------

## 节点检查

执行：

    ros2 node list

应该包含：

    /robot_state_publisher
    /joint_state_publisher_gui
    /rviz2

------------------------------------------------------------------------

## topic检查

执行：

    ros2 topic list

应该包含：

    /joint_states
    /robot_description
    /tf
    /tf_static

------------------------------------------------------------------------

## GUI测试

执行：

    ros2 topic echo /joint_states

拖动GUI滑杆。

应该看到：

joint name:

    fl_1

对应：

position变化。

------------------------------------------------------------------------

# 7. 不要做的事情

禁止：

-   修改URDF结构
-   修改mesh路径格式
-   删除joint
-   重命名link
-   引入Gazebo依赖修改description包
-   修改h723_bridge接口

当前任务只是：

修复 ROS2 description 显示链路。

------------------------------------------------------------------------

# 输出要求

完成后输出：

1.  修改了哪些文件
2.  为什么原来会报错
3.  如何验证成功
4.  后续如何连接 h723_bridge
