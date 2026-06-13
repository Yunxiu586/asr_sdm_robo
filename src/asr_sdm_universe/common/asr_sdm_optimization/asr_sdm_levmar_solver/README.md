# asr_sdm_levmar_solver

ROS 2 wrapper around [levmar](http://www.ics.forth.gr/~lourakis/levmar) 2.6 — a Levenberg-Marquardt non-linear least squares solver.

## Package layout

```sh
asr_sdm_levmar_solver/
├── CMakeLists.txt
├── package.xml
├── include/asr_sdm_levmar_solver/levmar.hpp
├── src/levmar/              # levmar 2.6 C library sources
├── src/levmar_wrapper.cpp
└── test/levmar_test.cpp
```

## Build & run

From the `levmar-2.6` workspace root:

```sh
colcon build --paths asr_sdm_levmar_solver
source install/setup.bash
ros2 run asr_sdm_levmar_solver levmar_test
```

## Use from another package

`package.xml`:

```xml
<depend>asr_sdm_levmar_solver</depend>
```

`CMakeLists.txt`:

```cmake
find_package(asr_sdm_levmar_solver REQUIRED)
target_link_libraries(your_target asr_sdm_levmar_solver::asr_sdm_levmar_solver)
```

## License

The levmar library is GPL-2.0-or-later. See `../LICENSE` in the workspace root.
