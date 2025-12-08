# project_GPS_IMU_1111

This repository contains an implementation and reference materials for fusing GNSS and IMU data using an Extended Kalman Filter (EKF) for navigation. It includes C/C++ algorithm code, data parsers, MATLAB visualization scripts, Python utilities, and multiple real-world test for validation and research.

What’s inside
- **Algorithms**: EKF-based navigation implementation (position, velocity, attitude estimation).
- **Data parsers**: GNSS and IMU parsers in C, C++ and Python.
- **Test data**: Multiple walk/ride/drive datasets for testing and visualization.
- **Scripts**: MATLAB scripts for plotting and validation; Python scripts for preprocessing.

Key features
- Parsing and time-alignment of raw GNSS and IMU data.
- Reference EKF implementation.
- Example MATLAB visualizations and multiple real-world test logs.

Quick start

1) Requirements

- A C++ toolchain (GCC/Clang)
- CMake (recommended >= 3.15)
- MATLAB (optional, for scripts under `algo/matlab`)
- Python (optional, for scripts under `src/drivers/*/python`)

1) Build

Linux/macOS:

```bash
cd project_GPS_IMU_1111/build
./build.sh
```

After building, executables and targets are placed under `build/` . Look for targets `EKF_NAV_INS` in the build output.

1) Run example (after build)

Linux/macOS:

```bash
./EKF_NAV_INS
```

Repository layout

- `CMakeLists.txt`  : Top-level CMake build configuration
- `src/`           : Source code
	- `app/`         : Application entry (`main_ekf_nav_ins.cpp`)
	- `algorithms/`  : Algorithm implementations (`ekfNavIns`)
	- `drivers/`     : GNSS/IMU parsers and drivers (C/C++/Python)
	- `utils/`       : Utility functions
- `algo/`          : Reference implementations and MATLAB scripts
	- `matlab/`      : MATLAB plotting and simulation scripts
	- `c&c++/`       : Reference C/C++ implementations
	- `test/`        : real-world test (walk/drive/cycle)
- `test/`          : Unit tests
- `build/`         : Build script
- `LICENSE`        : License file

Test data and examples

The main real-world test are under `algo/test/`, containing multiple scenarios (walking, cycling, driving). Use the MATLAB scripts in `algo/matlab` to plot tracks and compare EKF outputs.

How to contribute

- Fork the repository and create a feature branch.
- Open a clear Pull Request with a description of changes and any test data if applicable.

Running unit tests

From the build directory you can run:

```bash
cd project_GPS_IMU_1111/test/build
./build.sh
```

License & contact

- See the `LICENSE` file in the repository root for license terms.
- Please open Issues for bugs, feature requests, or contact the repository owner via Git history if needed.

More information

For algorithm details of the EKF, see the MATLAB annotated scripts in `algo/matlab`.

Contributions and improvements are welcome!
