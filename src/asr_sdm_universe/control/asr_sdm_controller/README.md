# asr_sdm_controller

`asr_sdm_controller` 包包含前端跟随控制器、离线 2D 仿真可视化、实时 ROS2 控制节点和实时可视化节点。

## 依赖

可视化依赖 Python matplotlib 运行环境：

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib python3-tk
```

手柄输入依赖 ROS2 `joy` 包：

```bash
sudo apt install ros-${ROS_DISTRO}-joy
```

## 编译

第一次编译连同依赖一起编译：

```bash
cd ~/asr_sdm_robo
colcon build --packages-up-to asr_sdm_controller
source install/setup.bash
```


## 离线 2D 仿真可视化

离线程序不依赖 ROS2 topic，内部使用固定速度命令序列，适合检查控制器跟随效果。

```bash
ros2 run asr_sdm_controller front_unit_following_controller_test_2d
```

该程序使用：

- `FrontUnitFollowingController`
- `RobotModel`
- `matplotlibcpp`

仿真控制周期：

```text
20 ms
```

## 手柄实时控制与可视化

实时链路如下：

```text
/dev/input/js0
  -> joy_node (/joy)
  -> asr_sdm_teleop_node (/asr_sdm/cmd_vel)
  -> realtime_front_unit_controller
       -> /asr_sdm/control_cmd
       -> /asr_sdm/controller_state
  -> realtime_controller_visualizer
```
### 四个终端

### 1. 启动手柄 joy 节点

```bash
source install/setup.bash
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0
```

检查手柄输入：

```bash
ros2 topic echo /joy
```

### 2. 启动 teleop 节点

```bash
source install/setup.bash
ros2 run asr_sdm_teleop asr_sdm_teleop_node
```

默认映射：

| 手柄输入 | 输出 |
|---|---|
| 左摇杆前推 | `/asr_sdm/cmd_vel.linear.x` |
| 右摇杆左右 | `/asr_sdm/cmd_vel.angular.z` |

默认比例：

```text
linear_scale = 0.12
angular_scale = 0.5
```

检查 teleop 输出：

```bash
ros2 topic echo /asr_sdm/cmd_vel
```

### 3. 启动实时控制节点

```bash
source install/setup.bash
ros2 run asr_sdm_controller realtime_front_unit_controller
```

该节点订阅：

```text
/asr_sdm/cmd_vel
```

发布：

```text
/asr_sdm/control_cmd
/asr_sdm/controller_state
```

检查控制输出：

```bash
ros2 topic echo /asr_sdm/control_cmd
ros2 topic echo /asr_sdm/controller_state
```

### 4. 启动实时可视化节点

```bash
source install/setup.bash
ros2 run asr_sdm_controller realtime_controller_visualizer
```

关闭 matplotlib 窗口后，可视化节点会自动退出。

## topic 说明

| Topic | 类型 | 说明 |
|---|---|---|
| `/joy` | `sensor_msgs/msg/Joy` | `joy_node` 读取手柄后发布 |
| `/asr_sdm/cmd_vel` | `geometry_msgs/msg/Twist` | teleop 输出的头部速度 |
| `/asr_sdm/control_cmd` | `asr_sdm_control_msgs/msg/ControlCmd` | 实时控制节点输出给硬件的控制命令 |
| `/asr_sdm/controller_state` | `std_msgs/msg/Float64MultiArray` | 实时可视化使用的控制器内部状态 |

## 主要可执行目标

| 命令 | 作用 |
|---|---|
| `ros2 run asr_sdm_controller front_unit_following_controller_test_2d` | 离线 2D 仿真可视化 |
| `ros2 run asr_sdm_controller realtime_front_unit_controller` | 实时控制节点 |
| `ros2 run asr_sdm_controller realtime_controller_visualizer` | 实时可视化节点 |
| `ros2 run asr_sdm_teleop asr_sdm_teleop_node` | 手柄到速度命令转换节点 |
