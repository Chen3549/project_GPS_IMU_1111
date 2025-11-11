#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <vector>
#include <string>
#include <regex>
#include <cmath>
#include "gps_parse.h"

size_t g_gps_msg_id = 0;
GPS_MSG_S g_gps_curr_msg;
std::queue<GPS_MSG_S> g_gps_msg_queue;
std::mutex g_gps_msg_queue_mtx;

std::queue<GPS_MSG_S>& get_gps_msg_queue()
{
    return g_gps_msg_queue;
}

std::mutex& get_gps_msg_queue_mtx()
{
    return g_gps_msg_queue_mtx;
}

void push_gps_msg()
{
    std::lock_guard<std::mutex> lock(g_gps_msg_queue_mtx);
    g_gps_curr_msg.msg_id = g_gps_msg_id++;
    g_gps_msg_queue.push(g_gps_curr_msg);

    return;
}

std::optional<GPS_MSG_S> pop_gps_msg()
{
    std::lock_guard<std::mutex> lock(g_gps_msg_queue_mtx);
    if (!g_gps_msg_queue.empty()) {
        GPS_MSG_S gps_msg = g_gps_msg_queue.front();
        g_gps_msg_queue.pop();
        return gps_msg;
    } 
    return std::nullopt;
}

static int open_serial_port(const char* device, speed_t baud_rate)
{
    // 打开串口设备
    int serial_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
        std::cerr << "Error opening serial port" << std::endl;
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

size_t read_fixed_len(int serial_port, void *buffer, size_t buffer_len)
{
    size_t read_total = 0;
    while (read_total < buffer_len) {
        int read_bytes = read(serial_port, (char *)buffer + read_total, buffer_len - read_total);
        if (read_bytes < 0) {
            std::cerr << "Error occurred while reading data!" << std::endl;
            return read_bytes;
        }
        read_total += read_bytes;
    }
    return buffer_len;
}

std::vector<std::string> format_gps_message(std::string &message)
{
    size_t comma_pos = message.find(',');
    // remove prefix "$GNxxx,"
    if (comma_pos != std::string::npos) {
        message.erase(0, comma_pos + 1);
    }
    // remove suffix \r\n
    if (message.size() >= 2 && message.substr(message.size() - 2) == "\r\n") {
        message.erase(message.end() - 2, message.end());
    }

    std::vector<std::string> result;
    std::string current;
    for (size_t i = 0; i < message.size(); ++i) {
        char c = message[i];
        if (c == ',') {
            if (current.empty()) {
                current = "000000.000";
            }
            result.push_back(current); // 即使 current 为空也要 push_back
            current.clear();
        } else {
            current += c;
        }
    }
    result.push_back(current); // 最后一个字段
    return result;
} 

void parse_gps_GNGGA(std::string &message)
{
    std::vector<std::string> GNGGA_fields = format_gps_message(message);

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    g_gps_curr_msg.utc.time = static_cast<uint64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
    if (true) {
        auto now = std::chrono::system_clock::now();
        // 转换为time_t类型（秒级精度），用于获取日期时间分量
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        // 转换为本地时间
        std::tm* local_time = std::localtime(&now_time);
        // 提取小时、分钟、秒（0-23, 0-59, 0-59）
        g_gps_curr_msg.utc.hour = local_time->tm_hour;
        g_gps_curr_msg.utc.min = local_time->tm_min;
        g_gps_curr_msg.utc.sec = local_time->tm_sec;
        // 计算毫秒（0-999）
        g_gps_curr_msg.utc.ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch() % std::chrono::seconds(1)
        ).count();
    } else {
        g_gps_curr_msg.utc.hour = std::stoi(GNGGA_fields[0].substr(0, 2));
        g_gps_curr_msg.utc.min = std::stoi(GNGGA_fields[0].substr(2, 2));
        g_gps_curr_msg.utc.sec = std::stoi(GNGGA_fields[0].substr(4, 2));
        g_gps_curr_msg.utc.ms = std::stoi(GNGGA_fields[0].substr(7, 3));
    }
    // if (hour < 0 || hour >= 24 || minute < 0 || minute >= 60 || 
    //     second < 0 || second >= 60 || ms < 0 || ms >= 1000) {
    //     std::cerr << "UTC value invalid." std::endl;
    // }

    g_gps_curr_msg.gngga.lat_ang = std::stoi(GNGGA_fields[1].substr(0, 2)) + std::stod(GNGGA_fields[1].substr(2)) / 60.0;
    g_gps_curr_msg.gngga.lat_rad = g_gps_curr_msg.gngga.lat_ang * M_PI / 180.0;
    if (GNGGA_fields[2] != "N") {
        g_gps_curr_msg.gngga.lat_ang = -g_gps_curr_msg.gngga.lat_ang;
        g_gps_curr_msg.gngga.lat_rad = -g_gps_curr_msg.gngga.lat_rad;
    }

    g_gps_curr_msg.gngga.lon_ang = std::stoi(GNGGA_fields[3].substr(0, 3)) + std::stod(GNGGA_fields[3].substr(3)) / 60.0;
    g_gps_curr_msg.gngga.lon_rad = g_gps_curr_msg.gngga.lon_ang * M_PI / 180.0;
    if (GNGGA_fields[4] != "E") {
        g_gps_curr_msg.gngga.lon_ang = -g_gps_curr_msg.gngga.lon_ang;
        g_gps_curr_msg.gngga.lon_rad = -g_gps_curr_msg.gngga.lon_rad;
    }

    g_gps_curr_msg.gngga.msl = std::stod(GNGGA_fields[8]);

    return;
}

void parse_gps_GNVTG(std::string &message)
{
    std::vector<std::string> GNVTG_fields = format_gps_message(message);

    g_gps_curr_msg.gnvtg.kph = std::stod(GNVTG_fields[6]);
    g_gps_curr_msg.gnvtg.cogt_ang = std::stod(GNVTG_fields[0]);
    g_gps_curr_msg.gnvtg.cogt_rad = g_gps_curr_msg.gnvtg.cogt_ang * M_PI / 180.0;
    g_gps_curr_msg.gnvtg.kph_north = g_gps_curr_msg.gnvtg.kph * std::cos(g_gps_curr_msg.gnvtg.cogt_rad);
    g_gps_curr_msg.gnvtg.kph_east = g_gps_curr_msg.gnvtg.kph * std::sin(g_gps_curr_msg.gnvtg.cogt_rad);
    g_gps_curr_msg.gnvtg.kph_up = 0;

    push_gps_msg();
    if (false) {
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << "Time: " << g_gps_curr_msg.utc.time << std::endl;
        std::cout << "UTC: " << g_gps_curr_msg.utc.hour << ":" << g_gps_curr_msg.utc.min << ":" << g_gps_curr_msg.utc.sec << "." << g_gps_curr_msg.utc.ms << std::endl;
        std::cout << "lat_ang: " << g_gps_curr_msg.gngga.lat_ang << std::endl;
        std::cout << "lat_rad: " << g_gps_curr_msg.gngga.lat_rad << std::endl;
        std::cout << "lon_ang: " << g_gps_curr_msg.gngga.lon_ang << std::endl;
        std::cout << "lon_rad: " << g_gps_curr_msg.gngga.lon_rad << std::endl;
        std::cout << "msl: " << g_gps_curr_msg.gngga.msl << std::endl;
        std::cout << "kph: " << g_gps_curr_msg.gnvtg.kph << std::endl;
        std::cout << "kph_east: " << g_gps_curr_msg.gnvtg.kph_east << std::endl;
        std::cout << "kph_north: " << g_gps_curr_msg.gnvtg.kph_north << std::endl;
        std::cout << "kph_up: " << g_gps_curr_msg.gnvtg.kph_up << std::endl;
        std::cout << "cogt_ang: " << g_gps_curr_msg.gnvtg.cogt_ang << std::endl;
        std::cout << "cogt_rad: " << g_gps_curr_msg.gnvtg.cogt_rad << std::endl;
    }

    return;
}

void parse_gps_message(std::string &message)
{
    if (message.find("$GNGGA") == 0) {
        parse_gps_GNGGA(message);
    } else if (message.find("$GNVTG") == 0) {
        parse_gps_GNVTG(message);
    }
    return;
}

#define GPS_PARSE_LEN 50

void read_gps_data()
{
    const char* serial_device = "/dev/ttyUSB1";
    // Baud rate, depending on the actual device settings, commonly includes B9600, B115200, etc.
    speed_t baud_rate = B115200;

    int serial_port = open_serial_port(serial_device, baud_rate);
    if (serial_port == -1) {
        std::cerr << "Open port failed." << std::endl;
        return;
    }

    std::cout << "Start reading GPS serial data (Press Ctrl+C to exit)..." << std::endl;
    std::string data;
    char buffer[GPS_PARSE_LEN + 1];
    while (true) {
        ssize_t buffer_len = read_fixed_len(serial_port, buffer, GPS_PARSE_LEN);
        if (buffer_len < 0) {
            std::cerr << "Reading buffer failed.";
            return;
        }
        buffer[buffer_len] = '\0';
        data.append(buffer);

        size_t start, end;
        while ((start = data.find("$")) != std::string::npos) {
            end = data.find("\r\n", start);
            if (end == std::string::npos) {
                break;
            }

            std::string message = data.substr(start, end - start + 2);
            parse_gps_message(message);
            data.erase(0, end + 2);
        }

        // 限制缓冲区长度，防止无限增长
        // if (data.size() > 1024) {
        //     data.erase(0, data.size() - 512);
        // } else {
        //     usleep(10000); // 防止CPU占用过高
        // }
    }

    // close(serial_port);
    return;
}

// int main()
// {
//     read_gps_data();
// 
//     return 0;
// }
