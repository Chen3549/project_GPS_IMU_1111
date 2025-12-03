#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <termios.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t ts_us;
    int hour;
    int min;
    int sec;
    int ms;
} SENSOR_TIME_S;

int open_serial_port(const char* device, speed_t baud_rate);

#ifdef __cplusplus
}
#endif

#endif
