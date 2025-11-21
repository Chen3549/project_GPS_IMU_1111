本项目为扩展卡尔曼滤波(EKF)的matlab实现

## 1. 目录结构
目录结构如下所示
```
EKF
  |- c&c++
  |    |- ekfNavINS.cpp      % ekf核心文件
  |    |- ekfNavINS.h        % ekf头文件
  |    |_ EKF_NAV_INS说明.md % 说明文档
  |- matlab
  |    |- ekfNavINS.m             % ekf核心文件
  |    |- draw_ekf_data.m         % ekf绘图工具
  |    |_ test_ekf.m              % ekf使用案例
  |_ test
       |- drive_test          % 驾车测试
       |- cycle_test          % 骑行测试
       |_ walk_test           % 行走测试
```
## 2. 运行
### 2.1 C/C++
C/C++版本EKF库编译、运行参照EKF_NAV_INS说明.md
### 2.2 MATLAB
MATLAB版本运行EKF库使用案例运行

读取GPS+IMU数据，调用EKF进行仿真处理，绘制图形
```matlab
>> test_ekf
```

调用绘图工具, 对EKF结果数据进行处理、绘制
```matlab
>> draw_ekf_data('./test/walk_test/2025_11_12_11_10_cloudy/data/ekf_2025_11_12_11_10_57.csv', true, './test/walk_test/2025_11_12_11_10_cloudy/picture/')
```