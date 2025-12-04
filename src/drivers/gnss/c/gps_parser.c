#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <pthread.h>
#include "gps_parser.h"

#define GPS_PARSE_LEN 200
#define MAX_FIELDS 32
#define MAX_FIELD_LEN 64

size_t g_gps_msg_id = 0;
GPS_MSG_S g_gps_curr_msg;
GPS_Queue g_gps_msg_queue = {NULL, NULL, 0};
pthread_mutex_t g_gps_msg_queue_mtx = PTHREAD_MUTEX_INITIALIZER;

GPS_Queue* get_gps_msg_queue(void) {
    return &g_gps_msg_queue;
}

pthread_mutex_t* get_gps_msg_queue_mtx(void) {
    return &g_gps_msg_queue_mtx;
}

// 队列操作函数
static void queue_push(GPS_MSG_S data) {
    Node* new_node = (Node*)malloc(sizeof(Node));
    if (!new_node) {
        fprintf(stderr, "Memory allocation failed\n");
        return;
    }
    new_node->data = data;
    new_node->next = NULL;

    if (g_gps_msg_queue.rear == NULL) {
        g_gps_msg_queue.front = g_gps_msg_queue.rear = new_node;
    } else {
        g_gps_msg_queue.rear->next = new_node;
        g_gps_msg_queue.rear = new_node;
    }
    g_gps_msg_queue.size++;
}

static bool queue_pop(GPS_MSG_S* data) {
    if (g_gps_msg_queue.front == NULL) {
        return false;
    }

    Node* temp = g_gps_msg_queue.front;
    *data = temp->data;
    g_gps_msg_queue.front = g_gps_msg_queue.front->next;

    if (g_gps_msg_queue.front == NULL) {
        g_gps_msg_queue.rear = NULL;
    }

    free(temp);
    g_gps_msg_queue.size--;
    return true;
}

void push_gps_msg(void) {
    pthread_mutex_lock(&g_gps_msg_queue_mtx);
    g_gps_curr_msg.msg_id = g_gps_msg_id++;
    queue_push(g_gps_curr_msg);
    pthread_mutex_unlock(&g_gps_msg_queue_mtx);
}

bool pop_gps_msg(GPS_MSG_S* msg) {
    pthread_mutex_lock(&g_gps_msg_queue_mtx);
    bool result = queue_pop(msg);
    pthread_mutex_unlock(&g_gps_msg_queue_mtx);
    return result;
}

static int format_gps_message(char* message, char fields[][MAX_FIELD_LEN], int max_fields) {
    char* comma_pos = strchr(message, ',');
    if (comma_pos != NULL) {
        // memmove(message, comma_pos + 1, strlen(comma_pos + 1) + 1);
        message = comma_pos + 1;
    }

    size_t len = strlen(message);
    if (len >= 2 && message[len - 2] == '\r' && message[len - 1] == '\n') {
        message[len - 2] = '\0';
    }

    int field_count = 0;
    char current[MAX_FIELD_LEN] = { 0 };
    memset(current, 0, MAX_FIELD_LEN);
    size_t j = 0;
    for (size_t i = 0; (i < strlen(message)) && (field_count < max_fields); i++) {
        if ((message[i] == ',') || (message[i] == '*')) {
            if (strlen(current) == 0) {
                strncpy(current, "000000.000", MAX_FIELD_LEN);
            }
            strncpy(fields[field_count], current, MAX_FIELD_LEN);
            field_count++;
            memset(current, 0, MAX_FIELD_LEN);
            j = 0;
        } else {
            current[j] = message[i];
            j++;
        }
    }
    if (field_count < max_fields) {
        strncpy(fields[field_count], current, MAX_FIELD_LEN);
        field_count++;
    }
    return field_count;
}

static void parse_gps_GNGGA(char* message) {
    char GNGGA_fields[MAX_FIELDS][MAX_FIELD_LEN];
    int field_count = format_gps_message(message, GNGGA_fields, MAX_FIELDS);
    // if (field_count < 9) {
    //     fprintf(stderr, "Invalid GNGGA message format\n");
    //     return;
    // }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    g_gps_curr_msg.time.ts_us = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;

    if (true) {  // 保持原逻辑的条件判断
        time_t now_time;
        struct tm* local_time;
        time(&now_time);
        local_time = localtime(&now_time);
        
        g_gps_curr_msg.time.hour = local_time->tm_hour;
        g_gps_curr_msg.time.min = local_time->tm_min;
        g_gps_curr_msg.time.sec = local_time->tm_sec;
        
        struct timeval tv;
        gettimeofday(&tv, NULL);
        g_gps_curr_msg.time.ms = tv.tv_usec / 1000;
    } else {
        if (strlen(GNGGA_fields[0]) >= 9) {
            char hour_str[3], min_str[3], sec_str[3], ms_str[4];
            strncpy(hour_str, GNGGA_fields[0], 2);
            hour_str[2] = '\0';
            strncpy(min_str, GNGGA_fields[0] + 2, 2);
            min_str[2] = '\0';
            strncpy(sec_str, GNGGA_fields[0] + 4, 2);
            sec_str[2] = '\0';
            strncpy(ms_str, GNGGA_fields[0] + 7, 3);
            ms_str[3] = '\0';
            
            g_gps_curr_msg.time.hour = atoi(hour_str);
            g_gps_curr_msg.time.min = atoi(min_str);
            g_gps_curr_msg.time.sec = atoi(sec_str);
            g_gps_curr_msg.time.ms = atoi(ms_str);
        }
    }

    if (strlen(GNGGA_fields[1]) > 0) {
        char* dot_pos = strchr(GNGGA_fields[1], '.');
        if (dot_pos) {
            int degrees = atoi(GNGGA_fields[1]) / 100;
            double minutes = atof(dot_pos - 2);  // 假设格式为 ddmm.mmmm 或 dddmm.mmmm
            g_gps_curr_msg.gngga.lat_ang = degrees + minutes / 60.0;
        }
        g_gps_curr_msg.gngga.lat_rad = g_gps_curr_msg.gngga.lat_ang * M_PI / 180.0;
        
        if (strcmp(GNGGA_fields[2], "N") != 0) {
            g_gps_curr_msg.gngga.lat_ang = -g_gps_curr_msg.gngga.lat_ang;
            g_gps_curr_msg.gngga.lat_rad = -g_gps_curr_msg.gngga.lat_rad;
        }
    }

    if (strlen(GNGGA_fields[3]) > 0) {
        char* dot_pos = strchr(GNGGA_fields[3], '.');
        if (dot_pos) {
            int degrees = atoi(GNGGA_fields[3]) / 100;
            double minutes = atof(dot_pos - 2);
            g_gps_curr_msg.gngga.lon_ang = degrees + minutes / 60.0;
        }
        g_gps_curr_msg.gngga.lon_rad = g_gps_curr_msg.gngga.lon_ang * M_PI / 180.0;
        
        if (strcmp(GNGGA_fields[4], "E") != 0) {
            g_gps_curr_msg.gngga.lon_ang = -g_gps_curr_msg.gngga.lon_ang;
            g_gps_curr_msg.gngga.lon_rad = -g_gps_curr_msg.gngga.lon_rad;
        }
    }

    g_gps_curr_msg.gngga.msl = atof(GNGGA_fields[8]);
}

static void parse_gps_GNVTG(char* message) {
    char GNVTG_fields[MAX_FIELDS][MAX_FIELD_LEN];
    int field_count = format_gps_message(message, GNVTG_fields, MAX_FIELDS);
    // if (field_count < 7) {
    //     fprintf(stderr, "Invalid GNVTG message format\n");
    //     return;
    // }

    g_gps_curr_msg.gnvtg.kph = atof(GNVTG_fields[6]);
    g_gps_curr_msg.gnvtg.cogt_ang = atof(GNVTG_fields[0]);
    g_gps_curr_msg.gnvtg.cogt_rad = g_gps_curr_msg.gnvtg.cogt_ang * M_PI / 180.0;
    g_gps_curr_msg.gnvtg.kph_north = g_gps_curr_msg.gnvtg.kph * cos(g_gps_curr_msg.gnvtg.cogt_rad);
    g_gps_curr_msg.gnvtg.kph_east = g_gps_curr_msg.gnvtg.kph * sin(g_gps_curr_msg.gnvtg.cogt_rad);
    g_gps_curr_msg.gnvtg.kph_up = 0;

    // push_gps_msg();

    if (false) {
        printf("--------------------------------------------------\n");
        printf("msg_id: %zu\n", g_gps_curr_msg.msg_id);
        printf("Time: %llu\n", (unsigned long long)g_gps_curr_msg.time.ts_us);
        printf("UTC: %d:%d:%d.%d\n", 
               g_gps_curr_msg.time.hour,
               g_gps_curr_msg.time.min,
               g_gps_curr_msg.time.sec,
               g_gps_curr_msg.time.ms);
        printf("lat_ang: %f\n", g_gps_curr_msg.gngga.lat_ang);
        printf("lat_rad: %f\n", g_gps_curr_msg.gngga.lat_rad);
        printf("lon_ang: %f\n", g_gps_curr_msg.gngga.lon_ang);
        printf("lon_rad: %f\n", g_gps_curr_msg.gngga.lon_rad);
        printf("msl: %f\n", g_gps_curr_msg.gngga.msl);
        printf("kph: %f\n", g_gps_curr_msg.gnvtg.kph);
        printf("kph_east: %f\n", g_gps_curr_msg.gnvtg.kph_east);
        printf("kph_north: %f\n", g_gps_curr_msg.gnvtg.kph_north);
        printf("kph_up: %f\n", g_gps_curr_msg.gnvtg.kph_up);
        printf("cogt_ang: %f\n", g_gps_curr_msg.gnvtg.cogt_ang);
        printf("cogt_rad: %f\n", g_gps_curr_msg.gnvtg.cogt_rad);
    }
}

void parse_gps_message(char* message) {
    if (strstr(message, "$GNGGA") == message) {
        parse_gps_GNGGA(message);
    } else if (strstr(message, "$GNVTG") == message) {
        parse_gps_GNVTG(message);
    }
}

void read_gps_data(void) {
    const char* serial_device = "/dev/ttyUSB0";
    speed_t baud_rate = B115200;

    int serial_port = open_serial_port(serial_device, baud_rate);
    if (serial_port == -1) {
        fprintf(stderr, "Open port failed.\n");
        return;
    }

    printf("Start reading GPS serial data (Press Ctrl+C to exit)...\n");
    char data[2048] = {0};
    char buffer[GPS_PARSE_LEN + 1];
    
    while (true) {
        ssize_t buffer_len = read(serial_port, buffer, GPS_PARSE_LEN);
        if (buffer_len < 0) {
            fprintf(stderr, "Reading buffer failed.");
            return;
        }
        buffer[buffer_len] = '\0';
        strncat(data, buffer, sizeof(data) - strlen(data) - 1);

        char* start, end;
        while ((start = strchr(data, '$')) != NULL) {
            char* end = strstr(start, "\r\n");
            if (!end) break;

            char message[GPS_PARSE_LEN + 1];
            strncpy(message, start, end - start + 2);
            message[end - start + 2] = '\0';

            parse_gps_message(message);

            memmove(data, end + 2, strlen(end + 2) + 1);
        }
    }

    close(serial_port);
}

// int main() {
//     read_gps_data();
//     return 0;
// }
