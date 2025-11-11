'''
1. Onece program started, recording IMU data and saving it in a file.
2. Press Enter to stop receiving IMU data.
3. Read recorded data and draw the chart.
'''
import datetime
import serial
import imu_parser

def save_data():
    current_time = datetime.datetime.now()
    time_str = current_time.strftime("%Y%m%d_%H%M%S")
    filename = f"{time_str}.txt"

    try:
        with open(filename, 'w', encoding='utf-8') as file:
            print("success.")
    except IOError as e:
        return

    return

import numpy as np
import math
import time

# 初始坐标（米）
position = np.array([0.0, 0.0, 0.0])  # x, y, z
velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz

# 时间间隔（秒）
dt = 0.1

def euler_to_rotation_matrix(roll, pitch, yaw):
    """将欧拉角（角度）转换为旋转矩阵"""
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    Ry = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    # 总旋转矩阵：R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    return R

def update_position(accel, euler_angles):
    global velocity, position

    # 获取旋转矩阵（本体坐标系到世界坐标系）
    R = euler_to_rotation_matrix(*euler_angles)

    # 重力矫正（简单起见假设z轴方向含重力）
    accel[2] -= 9.81  # 重力加速度校正（只针对竖直方向）

    # 变换到世界坐标系
    accel_world = R @ accel

    # 积分得速度、位置
    velocity += accel_world * dt
    position += velocity * dt

    return position.copy()


# 示例 IMU 数据序列（实际应用中为每0.1s读取新数据）
imu_data_stream = [
    # [ax, ay, az, gx, gy, gz, dx, dy, dz]
    [0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 静止
    [1.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # X方向加速
    [1.0, 0.0, 9.81, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0], # X方向加速 + 翻滚
    [0.0, 1.0, 9.81, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0], # Y方向加速 + 俯仰
    [0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0], # Z方向静止 + 偏航

    [-1.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # X方向加速
    [-1.0, 0.0, 9.81, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0], # X方向加速 + 翻滚
    [0.0, -1.0, 9.81, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0], # Y方向加速 + 俯仰
    [0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0], # Z方向静止 + 偏航

    [0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 静止
    [0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 静止
    [0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 静止
    [0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 静止
    [0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 静止
]

# 模拟运行
def pos_test():
    for i, imu in enumerate(imu_data_stream):
        ax, ay, az, gx, gy, gz, dx, dy, dz = imu
        accel = np.array([ax, ay, az])
        euler = (dx, dy, dz)
        
        pos = update_position(accel, euler)
        print(f"Time: {round((i+1)*dt, 2)}s, Position: {pos}")
        time.sleep(dt)  # 模拟时间间隔


if __name__ == "__main__":
    current_time = datetime.datetime.now()
    time_str = current_time.strftime("%Y%m%d_%H%M%S")
    filename = f"{time_str}.txt"
    with open(filename, 'w', encoding='utf-8') as pos_log:
        port = 'COM6' # USB serial port 
        baud = 9600   # Same baud rate as the INERTIAL navigation module
        ser = serial.Serial(port, baud, timeout=0.5)
        while(1):
            datahex = ser.read(33)
            imu = imu_parser.DueData(datahex)
            if imu is None:
                continue
            ax, ay, az, gx, gy, gz, dx, dy, dz = imu
            ax *= 9.81
            ay *= 9.81
            az *= 9.81
            accel = np.array([ax, ay, az])
            euler = (dx, dy, dz)
            pos = update_position(accel, euler)
            pos_log.write(str(pos.tolist())+"\n")
            print(pos)
