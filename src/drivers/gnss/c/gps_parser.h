#ifndef GPS_PARSE_H
#define GPS_PARSE_H

#include <stdint.h>
#include <stdbool.h>
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double lat_rad;
    double lat_ang;
    double lon_rad;
    double lon_ang;
    double msl; /* Mean Sea Level, Unit: m */
} GNGGA_S;

typedef struct {
    double kph; /* Unit: km/h */
    double kph_east;
    double kph_north;
    double kph_up;
    double cogt_ang; /* Course Over Ground True */
    double cogt_rad;
} GNVTG_S;

typedef struct {
    SENSOR_TIME_S time;
    GNGGA_S gngga;
    GNVTG_S gnvtg;
    size_t msg_id;
} GPS_MSG_S;

// 队列节点定义
typedef struct Node {
    GPS_MSG_S data;
    struct Node* next;
} Node;

// 队列结构
typedef struct {
    Node* front;
    Node* rear;
    size_t size;
} GPS_Queue;

// 全局队列及互斥锁
extern GPS_Queue g_gps_msg_queue;
extern pthread_mutex_t g_gps_msg_queue_mtx;

GPS_Queue* get_gps_msg_queue(void);
pthread_mutex_t* get_gps_msg_queue_mtx(void);

void read_gps_data(void);
bool pop_gps_msg(GPS_MSG_S* msg);

#ifdef __cplusplus
}
#endif

#endif
