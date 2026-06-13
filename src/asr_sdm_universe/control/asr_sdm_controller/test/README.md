# front_unit_following_controller_test_2d

## 目录结构

```
test/
├── README.md
├── front_unit_following_controller_test_2d.cpp
└── third_party/
    ├── matplotlibcpp.h          # vendored from matplotlib-cpp
    └── LICENSE.matplotlibcpp
```

## 依赖

- Python 3 开发包（`Python3::Python`）+ NumPy（`Python3::NumPy`）
- 运行时：`python3-matplotlib`、`python3-tk`（或其它可用的 matplotlib 后端）
- 图形界面（`$DISPLAY` 可用）

Ubuntu 24.04 上安装：

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib python3-tk
```

## 编译

在工作空间根目录 `~/asr_sdm_robo` 执行：

```bash

colcon build --packages-up-to asr_sdm_controller
```

## 运行

```bash
source install/setup.bash
ros2 run asr_sdm_controller front_unit_following_controller_test_2d
```

预期效果：弹出 1000×760 窗口，含 2×2 子图

1. 头部 / 三关节 / 尾部轨迹 + 当前 body
2. 当前 body shape
3. 三关节角 `phi1/phi2/phi3` 随时间曲线
4. 头部命令 `v` / `omega` 曲线

 `Ctrl+C` 或关闭窗口退出。
