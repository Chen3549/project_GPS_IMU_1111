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
  |    |- ekfNavINS.m        % ekf核心文件
  |    |- draw_ekf_data.m    % ekf绘图工具
  |    |_ test_ekf.m         % ekf使用案例
  |_ test
       |_ gnss.csv           % ekf结果
```
## 2. 运行
### 2.1 C/C++
C/C++版本EKF库编译、运行参照EKF_NAV_INS说明.md
### 2.2 MATLAB
MATLAB版本运行EKF库使用案例运行

% todo: 实现GPS+IMU仿真，调用EKF进行数据处理，保存EKF结果为.csv文件，并绘制图形
```matlab
>> test_ekf
```

MATLAB版本EKF运行实例命令
```matlab
>> draw_ekf_data('./test/gnss.csv', true)
```