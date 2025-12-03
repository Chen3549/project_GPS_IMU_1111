#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include "util.h"

int open_serial_port(const char* device, speed_t baud_rate)
{
    // 打开串口设备
    int serial_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
        //std::cerr << "Error opening serial port." << std::endl;
        printf("Error opening serial port.\r\n");
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
        //std::cerr << "Error setting serial port options." << std::endl;
        printf("Error setting serial port options.\r\n");
        close(serial_fd);
        exit(1);
    }

    // TCIFLUSH：清空输入缓冲；TCOFLUSH：清空输出缓冲；TCIOFLUSH：两者都清空
    // 若没有这一行会导致程序每次启动会有至少一条数据校验失败
    tcflush(serial_fd, TCIOFLUSH);

    return serial_fd;
}
