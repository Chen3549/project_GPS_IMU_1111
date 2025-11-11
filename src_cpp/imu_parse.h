#ifndef IMU_PARSE_H
#define IMU_PARSE_H

#include <queue>
#include <mutex>

/* Unit: g, available value: 9.8m/s2 */
struct IMU_ACC_S {
    double x;
    double y;
    double z;
};

/* Unit: degree/s */
struct IMU_GYRO_S {
    double x;
    double y;
    double z;
};

/* Unit: degree */
struct IMU_ANGLE_S {
    double x;
    double y;
    double z;
};

/* Unit: uT */
struct IMU_MAG_S {
    double x;
    double y;
    double z;
};

struct IMU_TIMESTAMP_S {
    uint64_t time;
    int hour;
    int min;
    int sec;
    int ms;
};

struct IMU_FRAME_S {
    IMU_ACC_S acc;
    IMU_GYRO_S gyro;
    IMU_ANGLE_S angle;
    IMU_MAG_S mag;
    IMU_TIMESTAMP_S timestamp;
    size_t frame_id;
};

std::queue<IMU_FRAME_S>& get_imu_frame_queue();
std::mutex& get_imu_frame_queue_mtx();

void read_imu_data();
std::optional<IMU_FRAME_S> pop_imu_frame();

#endif
