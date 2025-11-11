#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <queue>
#include <vector>
#include <optional>
#include <cmath> 
#include "imu_parse.h"
#include "gps_parse.h"
#include "ekfNavINS.h"

struct SENSOR_MSG_S {
    IMU_FRAME_S imu_frame;
    GPS_MSG_S gps_msg;
};

std::queue<SENSOR_MSG_S> g_sensor_msg_queue;
std::mutex g_sensor_msg_queue_mtx;

void push_sesor_msg(SENSOR_MSG_S sensor_msg)
{
    std::lock_guard<std::mutex> lock(g_sensor_msg_queue_mtx);
    g_sensor_msg_queue.push(sensor_msg);

    return;
}

std::optional<SENSOR_MSG_S> pop_sensor_msg()
{
    std::lock_guard<std::mutex> lock(g_sensor_msg_queue_mtx);
    if (!g_sensor_msg_queue.empty()) {
        SENSOR_MSG_S sensor_msg = g_sensor_msg_queue.front();
        g_sensor_msg_queue.pop();
        return sensor_msg;
    } 
    return std::nullopt;
}

struct EKF_MSG_S {
    double lat_ang;
    double lon_ang;
    double roll_ang;
    double pitch_ang;
    double heading_ang;
};

bool write_to_csv(const std::string& filename, const SENSOR_MSG_S& sensor_msg, const EKF_MSG_S& ekf_msg) {
    // 以追加模式打开文件（ios::app），若文件不存在则自动创建
    std::ofstream csv_file(filename, std::ios::app | std::ios::out);
    if (!csv_file.is_open()) {
        std::cerr << "错误：无法打开文件 " << filename << std::endl;
        return false;
    }

    // 第一次打开文件时，写入表头（判断文件是否为空）
    csv_file.seekp(0, std::ios::end); // 移动文件指针到末尾
    if (csv_file.tellp() == 0) { // 文件为空，写入表头，表头字段用逗号分隔
        csv_file << "frame_id,timestamp,time,"
                    "acc_x,acc_y,acc_z,"
                    "gyro_x,gyro_y,gyro_z,"
                    "angle_x,angle_y,angle_z,"
                    "mag_x,mag_y,mag_z,"
                    "gps_lat,gps_lon,gps_angle,gps_kph,gps_kph_east,gps_kph_north,gps_kph_up,"
                    "ekf_lat,ekf_lon,ekf_roll,ekf_pitch,ekf_heading" << std::endl;
    }

    // 写入坐标数据（字段顺序与表头一致）
    csv_file << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    csv_file << sensor_msg.imu_frame.frame_id << ","
             << sensor_msg.imu_frame.timestamp.time << ","
             << sensor_msg.imu_frame.timestamp.hour << ":"
             << sensor_msg.imu_frame.timestamp.min << ":"
             << sensor_msg.imu_frame.timestamp.sec << ":"
             << sensor_msg.imu_frame.timestamp.ms << ","
             << sensor_msg.imu_frame.acc.x << ","
             << sensor_msg.imu_frame.acc.y << ","
             << sensor_msg.imu_frame.acc.z << ","
             << sensor_msg.imu_frame.gyro.x << ","
             << sensor_msg.imu_frame.gyro.y << ","
             << sensor_msg.imu_frame.gyro.z << ","
             << sensor_msg.imu_frame.angle.x << ","
             << sensor_msg.imu_frame.angle.y << ","
             << sensor_msg.imu_frame.angle.z << ","
             << sensor_msg.imu_frame.mag.x << ","
             << sensor_msg.imu_frame.mag.y << ","
             << sensor_msg.imu_frame.mag.z << ","
             << sensor_msg.gps_msg.gngga.lat_ang << ","
             << sensor_msg.gps_msg.gngga.lon_ang << ","
             << sensor_msg.gps_msg.gnvtg.cogt_ang << ","
             << sensor_msg.gps_msg.gnvtg.kph << ","
             << sensor_msg.gps_msg.gnvtg.kph_east << ","
             << sensor_msg.gps_msg.gnvtg.kph_north << ","
             << sensor_msg.gps_msg.gnvtg.kph_up << ","
             << ekf_msg.lat_ang << ","
             << ekf_msg.lon_ang << ","
             << ekf_msg.roll_ang << ","
             << ekf_msg.pitch_ang << ","
             << ekf_msg.heading_ang << std::endl;

    // 刷新缓冲区，确保数据实时写入文件（避免程序崩溃导致数据丢失）
    csv_file.flush();
    csv_file.close();
    return true;
}

void ekf_fusion()
{
    // 1. 获取当前系统时间（时间戳，精确到秒）
    auto now = std::chrono::system_clock::now();
    // 2. 转换为 time_t 类型（兼容传统时间函数）
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    // 3. 转换为本地时间（避免UTC时差问题）
    std::tm local_tm = *std::localtime(&now_time);  // localtime 线程不安全，下文有优化方案
    // 4. 格式化字符串（拼接文件名）
    std::stringstream ekf_filename;
    ekf_filename << "ekf_"
                 << std::put_time(&local_tm, "%Y_%m_%d_%H_%M_%S")  // 核心格式化：年_月_日_时_分_秒
                 << ".csv";

    ekfNavINS ekf;
    while (true) {
        if (auto sensor_msg = pop_sensor_msg()) {
            if (false) {
                std::cout << "----------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << "msg_id: " << sensor_msg->gps_msg.msg_id << ". ";
                std::cout << "Time: " << sensor_msg->gps_msg.utc.time << std::endl;
                std::cout << "UTC: " << sensor_msg->gps_msg.utc.hour << ":" << sensor_msg->gps_msg.utc.min << ":" << sensor_msg->gps_msg.utc.sec << "." << sensor_msg->gps_msg.utc.ms << std::endl;
                // std::cout << "lat_ang: " << sensor_msg->gps_msg.gngga.lat_ang << std::endl;
                // std::cout << "lat_rad: " << sensor_msg->gps_msg.gngga.lat_rad << std::endl;
                // std::cout << "lon_ang: " << sensor_msg->gps_msg.gngga.lon_ang << std::endl;
                // std::cout << "lon_rad: " << sensor_msg->gps_msg.gngga.lon_rad << std::endl;
                // std::cout << "msl: " << sensor_msg->gps_msg.gngga.msl << std::endl;
                // std::cout << "kph: " << sensor_msg->gps_msg.gnvtg.kph << std::endl;
                // std::cout << "kph_east: " << sensor_msg->gps_msg.gnvtg.kph_east << std::endl;
                // std::cout << "kph_north: " << sensor_msg->gps_msg.gnvtg.kph_north << std::endl;
                // std::cout << "kph_up: " << sensor_msg->gps_msg.gnvtg.kph_up << std::endl;
                // std::cout << "cogt_ang: " << sensor_msg->gps_msg.gnvtg.cogt_ang << std::endl;
                // std::cout << "cogt_rad: " << sensor_msg->gps_msg.gnvtg.cogt_rad << std::endl;

                std::cout << "frame_id: " << sensor_msg->imu_frame.frame_id << ". ";
                std::cout << "Time: " << sensor_msg->imu_frame.timestamp.time << std::endl;
                std::cout << sensor_msg->imu_frame.timestamp.hour << ":" << sensor_msg->imu_frame.timestamp.min << ":" << sensor_msg->imu_frame.timestamp.sec << ":" << sensor_msg->imu_frame.timestamp.ms << " ";
                std::cout << "[acc_x,   acc_y,   acc_z  ]: " << sensor_msg->imu_frame.acc.x << ", " << sensor_msg->imu_frame.acc.y << ", " << sensor_msg->imu_frame.acc.z << ". ";
                std::cout << "[gyro_x,  gyro_y,  gyro_z ]: " << sensor_msg->imu_frame.gyro.x << ", " << sensor_msg->imu_frame.gyro.y << ", " << sensor_msg->imu_frame.gyro.z << ". ";
                std::cout << "[angle_x, angle_y, angle_z]: " << sensor_msg->imu_frame.angle.x << ", " << sensor_msg->imu_frame.angle.y << ", " << sensor_msg->imu_frame.angle.z << ". ";
                std::cout << std::endl;
                // std::cout << "[mag_x,   mag_y,   mag_z  ]: " << sensor_msg->imu_frame.mag.x << ", " << sensor_msg->imu_frame.mag.y << ", " << sensor_msg->imu_frame.mag.z << std::endl;
            } else {
                ekf.ekf_update(sensor_msg->imu_frame.timestamp.time,
                    sensor_msg->gps_msg.gnvtg.kph_north/3.6, sensor_msg->gps_msg.gnvtg.kph_east/3.6, -1*sensor_msg->gps_msg.gnvtg.kph_up/3.6,
                    sensor_msg->gps_msg.gngga.lat_rad, sensor_msg->gps_msg.gngga.lon_rad, sensor_msg->gps_msg.gngga.msl,
                    sensor_msg->imu_frame.gyro.x*M_PI/180.0, sensor_msg->imu_frame.gyro.y*M_PI/180.0, sensor_msg->imu_frame.gyro.z*M_PI/180.0,
                    sensor_msg->imu_frame.acc.x*9.794, sensor_msg->imu_frame.acc.y*9.794, sensor_msg->imu_frame.acc.z*9.794,
                    sensor_msg->imu_frame.mag.x*1e-3, sensor_msg->imu_frame.mag.y*1e-3, sensor_msg->imu_frame.mag.z*1e-3);
                std::cout << "----------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << std::dec << "msg_id: " << sensor_msg->gps_msg.msg_id << ". ";
                std::cout << "Time: " << sensor_msg->imu_frame.timestamp.time << std::endl;
                std::cout << "UTC: " << sensor_msg->imu_frame.timestamp.hour << ":" << sensor_msg->imu_frame.timestamp.min << ":" << sensor_msg->imu_frame.timestamp.sec << "." << sensor_msg->imu_frame.timestamp.ms << std::endl;
                std::cout << "gps_lat: " << sensor_msg->gps_msg.gngga.lat_ang << std::endl;
                std::cout << "gps_lon: " << sensor_msg->gps_msg.gngga.lon_ang << std::endl;
                std::cout << "ekf_lat : " << ekf.getLatitude_rad()*(180.0/M_PI) << std::endl;
                std::cout << "ekf_lon: " << ekf.getLongitude_rad()*(180.0/M_PI) << std::endl;

                EKF_MSG_S ekf_msg;
                ekf_msg.lat_ang = ekf.getLatitude_rad()*(180.0/M_PI);
                ekf_msg.lon_ang = ekf.getLongitude_rad()*(180.0/M_PI);
                ekf_msg.roll_ang = ekf.getRoll_rad()*(180.0/M_PI);
                ekf_msg.pitch_ang = ekf.getPitch_rad()*(180.0/M_PI);
                ekf_msg.heading_ang = ekf.getHeading_rad()*(180.0/M_PI);

                write_to_csv(ekf_filename.str(), *sensor_msg, ekf_msg);
            }
        }
    }
}

GPS_MSG_S gps_linear_interpolate(GPS_MSG_S &prev, GPS_MSG_S &curr, uint64_t inter_time)
{
    if (curr.utc.time == prev.utc.time) {
        return curr;
    }

    uint64_t delta_t = curr.utc.time - prev.utc.time;
    double ratio = (inter_time - prev.utc.time) / delta_t;

    GPS_MSG_S inter;

    inter.gngga.lat_rad = prev.gngga.lat_rad + ratio * (curr.gngga.lat_rad - prev.gngga.lat_rad);
    inter.gngga.lat_ang = prev.gngga.lat_ang + ratio * (curr.gngga.lat_ang - prev.gngga.lat_ang);
    inter.gngga.lon_rad = prev.gngga.lon_rad + ratio * (curr.gngga.lon_rad - prev.gngga.lon_rad);
    inter.gngga.lon_ang = prev.gngga.lon_ang + ratio * (curr.gngga.lon_ang - prev.gngga.lon_ang);
    inter.gngga.msl = prev.gngga.msl + ratio * (curr.gngga.msl - prev.gngga.msl);

    inter.gnvtg.kph = prev.gnvtg.kph + ratio * (curr.gnvtg.kph - prev.gnvtg.kph);
    inter.gnvtg.kph_east = prev.gnvtg.kph_east + ratio * (curr.gnvtg.kph_east - prev.gnvtg.kph_east);
    inter.gnvtg.kph_north = prev.gnvtg.kph_north + ratio * (curr.gnvtg.kph_north - prev.gnvtg.kph_north);
    inter.gnvtg.kph_up = prev.gnvtg.kph_up + ratio * (curr.gnvtg.kph_up - prev.gnvtg.kph_up);
    inter.gnvtg.cogt_ang = prev.gnvtg.cogt_ang + ratio * (curr.gnvtg.cogt_ang - prev.gnvtg.cogt_ang);
    inter.gnvtg.cogt_rad = prev.gnvtg.cogt_rad + ratio * (curr.gnvtg.cogt_rad - prev.gnvtg.cogt_rad);

    inter.utc.time = prev.utc.time;
    inter.utc.hour = prev.utc.hour;
    inter.utc.min = prev.utc.min;
    inter.utc.sec = prev.utc.sec;
    inter.utc.ms = prev.utc.ms;

    inter.msg_id = prev.msg_id;

    return inter;
}

// Preprocessing & Time Sync
void gps_imu_sensor_fusion()
{
    // 过滤启动之后GPS和IMU不同步的数据
    GPS_MSG_S prev_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = pop_gps_msg(); while (!opt.has_value()) { opt = pop_gps_msg(); } return opt.value(); }();
    GPS_MSG_S curr_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = pop_gps_msg(); while (!opt.has_value()) { opt = pop_gps_msg(); } return opt.value(); }();
    IMU_FRAME_S imu_frame = [&]() { std::optional<IMU_FRAME_S> opt = pop_imu_frame(); while (!opt.has_value()) { opt = pop_imu_frame(); } return opt.value(); }();

    while (prev_gps_msg.utc.time > imu_frame.timestamp.time) {
        imu_frame = [&]() { std::optional<IMU_FRAME_S> opt = pop_imu_frame(); while (!opt.has_value()) { opt = pop_imu_frame(); } return opt.value(); }();
    }

    while (curr_gps_msg.utc.time < imu_frame.timestamp.time) {
        prev_gps_msg = curr_gps_msg;
        curr_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = pop_gps_msg(); while (!opt.has_value()) { opt = pop_gps_msg(); } return opt.value(); }();
    }

    SENSOR_MSG_S sensor_msg;
    sensor_msg.gps_msg = gps_linear_interpolate(prev_gps_msg, curr_gps_msg, imu_frame.timestamp.time);
    sensor_msg.imu_frame = imu_frame;
    push_sesor_msg(sensor_msg);

    // 一次循环只弹出一个IMU
    while (true) {  
        imu_frame = [&]() { std::optional<IMU_FRAME_S> opt = pop_imu_frame(); while (!opt.has_value()) { opt = pop_imu_frame(); } return opt.value(); }();

        while (curr_gps_msg.utc.time < imu_frame.timestamp.time) {
            prev_gps_msg = curr_gps_msg;
            curr_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = pop_gps_msg(); while (!opt.has_value()) { opt = pop_gps_msg(); } return opt.value(); }();
        }

        SENSOR_MSG_S sensor_msg;
        sensor_msg.gps_msg = gps_linear_interpolate(prev_gps_msg, curr_gps_msg, imu_frame.timestamp.time);
        sensor_msg.imu_frame = imu_frame;
        push_sesor_msg(sensor_msg);
    }
}

// void gps_imu_sensor_fusion()
// {
//     GPS_MSG_S prev_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = pop_gps_msg(); while (!opt.has_value()) { opt = pop_gps_msg(); } return opt.value(); }();
//     GPS_MSG_S curr_gps_msg;
//     IMU_FRAME_S imu_frame;
//     while (true) {  
//         curr_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = pop_gps_msg(); while (!opt.has_value()) { opt = pop_gps_msg(); } return opt.value(); }();
//         imu_frame = [&]() { std::optional<IMU_FRAME_S> opt = pop_imu_frame(); while (!opt.has_value()) { opt = pop_imu_frame(); } return opt.value(); }();
// 
//         while (prev_gps_msg.utc.time > imu_frame.timestamp.time) {
//             imu_frame = [&]() { std::optional<IMU_FRAME_S> opt = pop_imu_frame(); while (!opt.has_value()) { opt = pop_imu_frame(); } return opt.value(); }();
//         }
// 
//         while (curr_gps_msg.utc.time < imu_frame.timestamp.time) {
//             prev_gps_msg = curr_gps_msg;
//             curr_gps_msg = [&]() { std::optional<GPS_MSG_S> opt = pop_gps_msg(); while (!opt.has_value()) { opt = pop_gps_msg(); } return opt.value(); }();
//         }
// 
//         SENSOR_MSG_S sensor_msg;
//         sensor_msg.gps_msg = gps_linear_interpolate(prev_gps_msg, curr_gps_msg, imu_frame.timestamp.time);
//         sensor_msg.imu_frame = imu_frame;
//         push_sesor_msg(sensor_msg);
// 
//         prev_gps_msg = curr_gps_msg;
//     }
// }

int main()
{
    std::thread read_imu_data_thread(read_imu_data);
    std::thread read_gps_data_thread(read_gps_data);
    std::thread gps_imu_sensor_fusion_thread(gps_imu_sensor_fusion);
    std::thread ekf_fusion_thread(ekf_fusion);

    if (read_imu_data_thread.joinable()) {
        read_imu_data_thread.join();
    }
    if (read_gps_data_thread.joinable()) {
        read_gps_data_thread.join();
    }
    if (gps_imu_sensor_fusion_thread.joinable()) {
        gps_imu_sensor_fusion_thread.join();
    }
    if (ekf_fusion_thread.joinable()) {
        ekf_fusion_thread.join();
    }

    return 0;
}
