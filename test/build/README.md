Method 1:
cmake ..
make
./unit_tests

Method 2:
| Execution Command       | Function Description                                                                 |
|-------------------------|-------------------------------------------------------------------------------------|
| `sh build.sh`           | Default incremental build: Only compiles changed code for faster speed (full build on first execution) |
| `sh build.sh clean`     | Cleanup operation: Deletes the `ekf_nav_ins_build` directory + the `EKF_NAV_INS` binary file in the current directory |
| `sh build.sh rebuild`   | Full rebuild: Executes `clean` to remove all files first, then compiles from scratch (resolves compilation issues caused by cache) |