## 1. brief introduction

这里存放定位行走测试的记录

每次测试记录放在一个文件夹内, 命名格式为 "时间_天气"。例如 "2025_11_27_10_01_sunny", 代表这次测试是在2025年11月27日10点01分进行的, 外部天气晴。每个测试文件夹包含两个子文件夹: "data", "picture"。"data" 用于存放数据文件, 包括有 GNSS 模块原始测量数据, IMU 模块原始测量数据, EKF 滤波处理数据。"picture" 用于存放 Matlab 对此次测量记录数据绘制的图片, 包括轨迹图, 加速度图, 角速度图等等。

## 2. 数据文件
### 2.1 GNSS 数据文件

GNSS 数据的文件为 "CSV" 格式, 命名为 "GNSS_模块型号.csv"。例如 "GNSS_ATGM332D-F8N-76.csv", 代表这次测试使用 "GNSS_ATGM332D-F8N-76" GNSS 模块进行。

GNSS 文件数据遵循以下规则:

||time|lat|lon|msl|kph|cogt|
|-|-|-|-|-|-|-|
|description|UTC 时间|纬度|经度|海拔|速度|真北航向角|
|unit|μs|degree|degree|m|km/h|degree|

### 2.2 IMU 数据文件

IMU 数据的文件为 "CSV" 格式，命名为 "IMU_模块型号.csv"。例如 "IMU_JY901B.csv", 代表这次测试使用 "JY901B" IMU 模块进行。

IMU 文件数据遵循以下规则:

||time|ax|ay|az|wx|wy|wz|hx|hy|hz|
|-|-|-|-|-|-|-|-|-|-|-|
|description|UTC 时间|X 轴加速度|Y 轴加速度|Z 轴加速度|X 轴角速度|Y 轴角速度|Z 轴角速度|X 轴地磁场分量|Y 轴地磁场分量|Z 轴地磁场分量|
|unit|μs|m/s^2|m/s^2|m/s^2|degree/s|degree/s|degree/s|μT|μT|μT|

### 2.3 EKF 处理结果文件

EKF 处理结果文件为 "CSV" 格式, 命名统一为 "EKF_result.csv"。处理算法基于 [EKF_IMU_GPS](https://github.com/balamuruganky/EKF_IMU_GPS/) 开源库。

EKF 文件结果数据遵循以下规则:

||frame_id|timestamp|time|ekf_lat|ekf_lon|ekf_roll|ekf_pitch|ekf_heading|
|-|-|-|-|-|-|-|-|-|
|description|数据点计数索引|UTC 时间|可读时间字符串|EKF 处理所得纬度|EKF 处理所得经度|EKF 处理所得横滚角|EKF 处理所得俯仰角|EKF 处理所得航向角|
|unit|/|μs|/|degree|degree|degree|degree|degree|