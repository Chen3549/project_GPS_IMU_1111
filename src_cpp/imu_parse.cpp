#include <iostream>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <tuple>
#include <optional>
#include <mutex>
#include <queue>
#include "imu_parse.h"

size_t g_imu_frame_id = 0;
IMU_FRAME_S g_imu_curr_frame;
std::queue<IMU_FRAME_S> g_imu_frame_queue;
std::mutex g_imu_frame_queue_mtx;

std::queue<IMU_FRAME_S>& get_imu_frame_queue()
{
    return g_imu_frame_queue;
}

std::mutex& get_imu_frame_queue_mtx()
{
    return g_imu_frame_queue_mtx;
}

std::optional<IMU_FRAME_S> pop_imu_frame()
{
    std::lock_guard<std::mutex> lock(g_imu_frame_queue_mtx);
    if (!g_imu_frame_queue.empty()) {
        IMU_FRAME_S imu_frame = g_imu_frame_queue.front();
        g_imu_frame_queue.pop();
        return imu_frame;
    } 
    return std::nullopt;
}

void push_imu_frame()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    g_imu_curr_frame.timestamp.time = static_cast<uint64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;

    auto now = std::chrono::system_clock::now();
    // 转换为time_t类型（秒级精度），用于获取日期时间分量
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    // 转换为本地时间
    std::tm* local_time = std::localtime(&now_time);
    // 提取小时、分钟、秒（0-23, 0-59, 0-59）
    g_imu_curr_frame.timestamp.hour = local_time->tm_hour;
    g_imu_curr_frame.timestamp.min = local_time->tm_min;
    g_imu_curr_frame.timestamp.sec = local_time->tm_sec;
    // 计算毫秒（0-999）
    g_imu_curr_frame.timestamp.ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch() % std::chrono::seconds(1)
    ).count();

    g_imu_curr_frame.frame_id = g_imu_frame_id++;

    std::lock_guard<std::mutex> lock(g_imu_frame_queue_mtx);
    g_imu_frame_queue.push(g_imu_curr_frame);

    if (false) {
        std::cout << "frame_id: " << g_imu_curr_frame.frame_id << std::endl;
        std::cout << "Time: " << g_imu_curr_frame.timestamp.time << std::endl;
        std::cout << g_imu_curr_frame.timestamp.hour << ":" << g_imu_curr_frame.timestamp.min << ":" << g_imu_curr_frame.timestamp.sec << ":" << g_imu_curr_frame.timestamp.ms << std::endl;
        std::cout << "[acc_x,   acc_y,   acc_z  ]: " << g_imu_curr_frame.acc.x << ", " << g_imu_curr_frame.acc.y << ", " << g_imu_curr_frame.acc.z << std::endl;
        std::cout << "[gyro_x,  gyro_y,  gyro_z ]: " << g_imu_curr_frame.gyro.x << ", " << g_imu_curr_frame.gyro.y << ", " << g_imu_curr_frame.gyro.z << std::endl;
        std::cout << "[angle_x, angle_y, angle_z]: " << g_imu_curr_frame.angle.x << ", " << g_imu_curr_frame.angle.y << ", " << g_imu_curr_frame.angle.z << std::endl;
        std::cout << "[mag_x,   mag_y,   mag_z  ]: " << g_imu_curr_frame.mag.x << ", " << g_imu_curr_frame.mag.y << ", " << g_imu_curr_frame.mag.z << std::endl;
    }

    return;
}

static int open_serial_port(const char* device, speed_t baud_rate)
{
    // 打开串口设备
    int serial_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
        std::cerr << "Error opening serial port" << serial_fd << std::endl;
        exit(1);
    }

    // 设置串口参数
    struct termios options;
    memset(&options, 0, sizeof(options));
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    if (tcsetattr(serial_fd, TCSANOW, &options) != 0) {
        std::cerr << "Error setting serial port options" << std::endl;
        close(serial_fd);
        exit(1);
    }

    // TCIFLUSH：清空输入缓冲；TCOFLUSH：清空输出缓冲；TCIOFLUSH：两者都清空
    // 若没有这一行会导致程序每次启动会有至少一条数据校验失败
    tcflush(serial_fd, TCIOFLUSH);

    return serial_fd;
}

std::tuple<double, double, double> get_acc(unsigned char *datahex)
{
    unsigned char axl = datahex[0];
    unsigned char axh = datahex[1];
    unsigned char ayl = datahex[2];
    unsigned char ayh = datahex[3];
    unsigned char azl = datahex[4];
    unsigned char azh = datahex[5];
    double k_acc = 16.0;
    double acc_x = (axh << 8 | axl) / 32768.0 * k_acc;
    double acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc;
    double acc_z = (azh << 8 | azl) / 32768.0 * k_acc;
    if (acc_x >= k_acc) {
        acc_x -= 2 * k_acc;
    }
    if (acc_y >= k_acc) {
        acc_y -= 2 * k_acc;
    }
    if (acc_z >= k_acc) {
        acc_z -= 2 * k_acc;
    }
    return {acc_x, acc_y, acc_z};
}

std::tuple<double, double, double> get_gyro(unsigned char *datahex)
{
    unsigned char wxl = datahex[0];
    unsigned char wxh = datahex[1];
    unsigned char wyl = datahex[2];
    unsigned char wyh = datahex[3];
    unsigned char wzl = datahex[4];
    unsigned char wzh = datahex[5];
    double k_gyro = 2000.0;
    double gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro;
    double gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro;
    double gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro;
    if (gyro_x >= k_gyro) {
        gyro_x -= 2 * k_gyro;
    }
    if (gyro_y >= k_gyro) {
        gyro_y -= 2 * k_gyro;
    }
    if (gyro_z >= k_gyro) {
        gyro_z -= 2 * k_gyro;
    }
    return {gyro_x, gyro_y, gyro_z};
}

std::tuple<double, double, double> get_angle(unsigned char *datahex)
{
    unsigned char rxl = datahex[0];
    unsigned char rxh = datahex[1];
    unsigned char ryl = datahex[2];
    unsigned char ryh = datahex[3];
    unsigned char rzl = datahex[4];
    unsigned char rzh = datahex[5];
    double k_angle = 180.0;
    double angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle;
    double angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle;
    double angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle;
    if (angle_x >= k_angle) {
        angle_x -= 2 * k_angle;
    }
    if (angle_y >= k_angle) {
        angle_y -= 2 * k_angle;
    }
    if (angle_z >= k_angle) {
        angle_z -= 2 * k_angle;
    }
    return {angle_x, angle_y, angle_z};
}

std::tuple<double, double, double> get_mag(unsigned char *datahex)
{
    unsigned char hxl = datahex[0];
    unsigned char hxh = datahex[1];
    unsigned char hyl = datahex[2];
    unsigned char hyh = datahex[3];
    unsigned char hzl = datahex[4];
    unsigned char hzh = datahex[5];
    double mag_x = static_cast<double>(static_cast<int16_t>((static_cast<uint16_t>(hxh) << 8) | hxl)) * 0.00667;
    double mag_y = static_cast<double>(static_cast<int16_t>((static_cast<uint16_t>(hyh) << 8) | hyl)) * 0.00667;
    double mag_z = static_cast<double>(static_cast<int16_t>((static_cast<uint16_t>(hzh) << 8) | hzl)) * 0.00667;
    return {mag_x, mag_y, mag_z};
}

#define IMU_FRAME_LEN 44
#define IMU_FIELD_LEN 11

int g_field_label = 0;
int g_field_pos = 0;
int g_check_sum = 0;
unsigned char g_acc_data[8] = {0}; 
unsigned char g_gyro_data[8] = {0}; 
unsigned char g_angle_data[8] = {0}; 
unsigned char g_mag_data[8] = {0}; 

void imu_parse(unsigned char* imu_data)
{
    for (int i = 0; i < IMU_FRAME_LEN; i++) {
        if (g_field_label == 0) {
            if ((imu_data[i] == 0x55) && (g_field_pos == 0)) {
                g_check_sum = imu_data[i];
                g_field_pos = 1;
            } else if ((imu_data[i] == 0x51) && (g_field_pos == 1)) {
                g_check_sum += imu_data[i];
                g_field_label = 1;
                g_field_pos = 2;
            } else if ((imu_data[i] == 0x52) && (g_field_pos == 1)) {
                g_check_sum += imu_data[i];
                g_field_label = 2;
                g_field_pos = 2;
            } else if ((imu_data[i] == 0x53) && (g_field_pos == 1)) {
                g_check_sum += imu_data[i];
                g_field_label = 3;
                g_field_pos = 2;
            } else if ((imu_data[i] == 0x54) && (g_field_pos == 1)) {
                g_check_sum += imu_data[i];
                g_field_label = 4;
                g_field_pos = 2;
            } else {
                g_check_sum = 0;
                g_field_pos = 0;
                g_field_label = 0;
            }
            // g_field_pos++; 这种写法不具有抗干扰性
        } else if (g_field_label == 1) { // acc
            if (g_field_pos < IMU_FIELD_LEN - 1) {
                g_acc_data[g_field_pos - 2] = imu_data[i];
                g_check_sum += imu_data[i];
                g_field_pos++;
            } else {
                if (imu_data[i] == (g_check_sum & 0xff)) {
                    auto [acc_x, acc_y, acc_z] = get_acc(g_acc_data);
                    // std::cout << "acc_x: " << acc_x << ", acc_y: " << acc_y << ", acc_z: " << acc_z << std::endl;
                    g_imu_curr_frame.acc.x = acc_x;
                    g_imu_curr_frame.acc.y = acc_y;
                    g_imu_curr_frame.acc.z = acc_z;
                } else {
                    std::cerr << "acc data check failed." << std::endl;
                }
                g_check_sum = 0;
                g_field_pos = 0;
                g_field_label = 0;
            }
        } else if (g_field_label == 2) { // gyro
            if (g_field_pos < IMU_FIELD_LEN - 1) {
                g_gyro_data[g_field_pos - 2] = imu_data[i];
                g_check_sum += imu_data[i];
                g_field_pos++;
            } else {
                if (imu_data[i] == (g_check_sum & 0xff)) {
                    auto [gyro_x, gyro_y, gyro_z] = get_gyro(g_gyro_data);
                    // std::cout << "gyro_x: " << gyro_x << ", gyro_y: " << gyro_y << ", gyro_z: " << gyro_z << std::endl;
                    g_imu_curr_frame.gyro.x = gyro_x;
                    g_imu_curr_frame.gyro.y = gyro_y;
                    g_imu_curr_frame.gyro.z = gyro_z;
                } else {
                    std::cout << "gyro data check failed." << std::endl;
                    std::cout << "-------------------------" << std::endl;
                    for (int j = 0; j <= i; j++) {
                        std::cout << std::setw(2) << std::setfill('0') << std::uppercase 
                            << std::hex << static_cast<int>(imu_data[j]) << " ";
                    }
                    std::cout << std::endl;
                    std::cout << "-------------------------" << std::endl;
                }
                g_check_sum = 0;
                g_field_pos = 0;
                g_field_label = 0;
            }
        } else if (g_field_label == 3) { // angle
            if (g_field_pos < IMU_FIELD_LEN - 1) {
                g_angle_data[g_field_pos - 2] = imu_data[i];
                g_check_sum += imu_data[i];
                g_field_pos++;
            } else {
                if (imu_data[i] == (g_check_sum & 0xff)) {
                    auto [angle_x, angle_y, angle_z] = get_angle(g_angle_data);
                    // std::cout << "angle_x: " << angle_x << ", angle_y: " << angle_y << ", angle_z: " << angle_z << std::endl;
                    g_imu_curr_frame.angle.x = angle_x;
                    g_imu_curr_frame.angle.y = angle_y;
                    g_imu_curr_frame.angle.z = angle_z;
                } else {
                    std::cout << "angle data check failed." << std::endl;
                }
                g_check_sum = 0;
                g_field_pos = 0;
                g_field_label = 0;
            }
        } else if (g_field_label == 4) { // magnet
            if (g_field_pos < IMU_FIELD_LEN - 1) {
                g_mag_data[g_field_pos - 2] = imu_data[i];
                g_check_sum += imu_data[i];
                g_field_pos++;
            } else {
                if (imu_data[i] == (g_check_sum & 0xff)) {
                    auto [mag_x, mag_y, mag_z] = get_mag(g_mag_data);
                    // std::cout << "mag_x: " << mag_x << ", mag_y: " << mag_y << ", mag_z: " << mag_z << std::endl;
                    g_imu_curr_frame.mag.x = mag_x;
                    g_imu_curr_frame.mag.y = mag_y;
                    g_imu_curr_frame.mag.z = mag_z;
                    push_imu_frame();
                } else {
                    std::cout << "mag data check failed." << std::endl;
                }
                g_check_sum = 0;
                g_field_pos = 0;
                g_field_label = 0;
            }
        }
    }
}

void read_imu_data()
{
    const char* serial_device = "/dev/ttyUSB0";
    // Baud rate, depending on the actual device settings, commonly includes B9600, B115200, etc.
    speed_t baud_rate = B115200;

    int serial_port = open_serial_port(serial_device, baud_rate);
    if (serial_port == -1) {
        return;
    }

    std::cout << "Start reading IMU serial data (Press Ctrl+C to exit)..." << std::endl;
    unsigned char buffer[IMU_FRAME_LEN] = { 0 };
    ssize_t bytesRead;
    bool readState = true;

    while (readState) {
        int totalRead = 0;
        while (totalRead < IMU_FRAME_LEN) {
            bytesRead = read(serial_port, buffer + totalRead, IMU_FRAME_LEN - totalRead);
            if (bytesRead < 0) {
                std::cerr << "Error occurred while reading data!" << std::endl;
                readState = false;
                break;
            }
            totalRead += bytesRead;
        }

        if (false) {
            for (ssize_t i = 0; i < IMU_FRAME_LEN; i++) {
                if (static_cast<int>(buffer[i]) == 0x55) {
                    std::cout << std::endl;
                }
                std::cout << std::setw(2) << std::setfill('0') << std::uppercase 
                        << std::hex << static_cast<int>(buffer[i]) << " ";
            }
        }

        imu_parse(buffer);
    }

    // close(serial_port);
}

// int main()
// {
//     read_imu_data();
//     return 0;
// }
