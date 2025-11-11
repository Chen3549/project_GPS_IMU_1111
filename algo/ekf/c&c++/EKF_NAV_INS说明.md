## 1. EKF_NAV_INS接口参数

### 1.1 输入
| 输入参数         | 单位 | 数据类型 | 说明 |
| ---------------- | ---- | ---- | ---- |
| Latitude         | rad  | double | 经度，范围为 -π ~ π，以本初子午线为分界，向东为正，向西为负。例如西经 30°，即为 -π/6 rad |
| Longitude        | rad  | double | 维度，范围为 -π/2 ~ π/2，以赤道为分界，向北为正，向南为负。例如南纬 60°，即为 -π/3 rad |
| Altitude         | m    | double | 海拔 |
| Velocity-North   | m/s  | double | 北向速度 |
| Velocity-East    | m/s  | double | 东向速度 |
| Velocity-Down    | m/s  | double | 对地垂直速度 |
| Accelarometer X  | m/s/s | float    | X 轴加速度分量 |
| Accelarometer Y | m/s/s | float | Y 轴加速度分量 |
| Accelarometer Z  | m/s/s | float | Z 轴加速度分量 |
| Gyro X           | rad/s | float | X 轴角速度，横滚，Roll。正值表示向右倾斜，负值表示向左倾斜 |
| Gyro Y           | rad/s | float | Y 轴角速度，俯仰，Pitch。正值表示抬头，负值表示低头 |
| Gyro Z           | rad/s | float | Z 轴角速度，偏航，Yaw |
| Magnetometer X   | mT   | float | X 轴地磁场分量 |
| Magnetometer Y   | mT   | float | Y 轴地磁场分量 |
| Magnetometer Z   | mT   | float | Z 轴地磁场分量 |

### 1.2 输出

| 输出参数       | 单位 | 数据类型 | 对应函数             | 说明                                                         |
| -------------- | ---- | -------- | -------------------- | ------------------------------------------------------------ |
| Latitude       | rad  | double   | getLatitude_rad()    | 经度，范围为 -π ~ π                                          |
| Longitude      | rad  | double   | getLongitude_rad()   | 维度，范围为 -π/2 ~ π/2                                      |
| Altitude       | m    | double   | getAltitude_m()      | 海拔                                                         |
| Velocity-North | m/s  | double   | getVelNorth_ms()     | 北向速度                                                     |
| Velocity-East  | m/s  | double   | getVelEast_ms()      | 东向速度                                                     |
| Velocity-Down  | m/s  | double   | getVelDown_ms()      | 对地垂直速度                                                 |
| Roll           | rad  | float    | getRoll_rad()        | 横滚角                                                       |
| Pitch          | rad  | float    | getPitch_rad()       | 俯仰角                                                       |
| Yaw            | rad  | float    | getHeading_rad()     | 航向角。使用 getHeading_rad() 获取原始航向角，未经过角度约束，可能随着时间累积超出 -π ~ π，例如持续顺时针旋转时，角度可能超过 π 。使用 getHeadingConstrainAngle180_rad() 获取约束在 -π ~ π 范围内的航向角 |
| GroundTrack    | rad  | float    | getGroundTrack_rad() | 地面轨迹方向，物体实际运动方向与北向的夹角                   |

### 1.3 偏置输出

| 输出参数   | 单位  | 数据类型 | 对应函数            | 说明                  |
| ---------- | ----- | -------- | ------------------- | --------------------- |
| GyroBiasX  | rad/s | float    | getGyroBiasX_rads() | 陀螺仪 X 轴偏差估计   |
| GyroBiasY  | rad/s | float    | getGyroBiasY_rads() | 陀螺仪 Y 轴偏差估计   |
| GyroBiasZ  | rad/s | float    | getGyroBiasZ_rads() | 陀螺仪 Z 轴偏差估计   |
| AccelBiasX | m/s/s | float    | getAccelBiasX_mss() | 加速度计 X 轴偏差估计 |
| AccelBiasY | m/s/s | float    | getAccelBiasY_mss() | 加速度计 Y 轴偏差估计 |
| AccelBiasZ | m/s/s | float    | getAccelBiasZ_mss() | 加速度计 Z 轴偏差估计 |

## 2. EKF_NAV_INS接口函数

ekfNavINS类public修饰的函数如下所示：

| 函数                                                         | 说明                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| void ekf_update( uint64_t time, double vn, double ve, double vd,double lat, double lon, double alt, float p, float q, float r, float ax, float ay, float az, float hx, float hy, float hz); | EKF 的主更新函数，接收多源传感器数据并执行滤波更新。time 为当前时间戳，vn/ve/vd 为北向/东向/垂向速度，lat/lon/alt 为经度/维度/海拔，p/q/r 为陀螺仪测量的 X/Y/Z 轴角速度，ax/ay/az 为加速度计测量的 X/Y/Z 轴加速度，hx/hy/hz 为磁力计测量的 X/Y/Z 轴磁场强度 |
| bool initialized();                                          | 判断 EKF 是否就绪                                            |
| float getPitch_rad();                                        | 获取俯仰角                                                   |
| float getRoll_rad();                                         | 获取横滚角                                                   |
| float getHeadingConstrainAngle180_rad();                     | 获取原始航向角                                               |
| float getHeading_rad();                                      | 获取约束在 -π ~ π 范围内的航向角                             |
| double getLatitude_rad();                                    | 获取纬度                                                     |
| double getLongitude_rad();                                   | 获取经度                                                     |
| double getAltitude_m();                                      | 获取海拔                                                     |
| double getVelNorth_ms();                                     | 获取北向速度                                                 |
| double getVelEast_ms();                                      | 获取东向速度                                                 |
| double getVelDown_ms();                                      | 获取垂向速度                                                 |
| float getGroundTrack_rad();                                  | 获取地面轨迹方向                                             |
| float getGyroBiasX_rads();                                   | 获取陀螺仪 X 轴偏差估计                                      |
| float getGyroBiasY_rads();                                   | 获取陀螺仪 Y 轴偏差估计                                      |
| float getGyroBiasZ_rads();                                   | 获取陀螺仪 Z 轴偏差估计                                      |
| float getAccelBiasX_mss();                                   | 获取加速度计 X 轴偏差估计                                    |
| float getAccelBiasY_mss();                                   | 获取加速度计 Y 轴偏差估计                                    |
| float getAccelBiasZ_mss();                                   | 获取加速度计 Z 轴偏差估计                                    |
| std::tuple<float,float,float> getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz); | 根据加速度计（ax, ay, az）和磁力计（hx, hy, hz）数据，计算俯仰角、横滚角、航向角，用于 EKF 初始化阶段的姿态初值计算，或者作为磁力计辅助修正姿态的补充方法 |
| void imuUpdateEKF(uint64_t time, imuData imu);               | 单独处理 IMU 数据的更新                                      |
| void gpsCoordinateUpdateEKF(gpsCoordinate coor);             | 单独处理 GPS 位置数据的更新                                  |
| void gpsVelocityUpdateEKF(gpsVelocity vel);                  | 单独处理 GPS 速度数据的更新                                  |

## 3. EKF_NAV_INS库编译

```bash
#安装eigen
#安装前先下载源码, 下载地址为 https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.gz
tar -zxvf eigen-3.4.1.tar.gz
cd eigen-3.4.1/
mkdir build
cd build/
cmake ..
sudo make install

#编译EKF_NAV_INS库
git clone https://github.com/balamuruganky/EKF_IMU_GPS
cd EKF_IMU_GPS/
git submodule update --init --recursive
cd ekf_nav_ins/
mkdir build
cd build/
cmake ..
make
```

