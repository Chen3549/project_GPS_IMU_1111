#ifndef GPS_PARSE_H
#define GPS_PARSE_H

#include <queue>
#include <mutex>
#include <optional>

struct UTC_S {
    uint64_t time;
    int hour;
    int min;
    int sec;
    int ms;
};

struct GNGGA_S {
    double lat_rad;
    double lat_ang;
    double lon_rad;
    double lon_ang;
    double msl; /* Mean Sea Level, Unit: m */
};

struct GNVTG_S {
    double kph; /* Unit: km/h */
    double kph_east;
    double kph_north;
    double kph_up;
    double cogt_ang; /* Course Over Ground True */
    double cogt_rad;
};

struct GPS_MSG_S {
    UTC_S utc;
    GNGGA_S gngga;
    GNVTG_S gnvtg;
    size_t msg_id;
};

std::queue<GPS_MSG_S>& get_gps_msg_queue();
std::mutex& get_gps_msg_queue_mtx();

void read_gps_data();
std::optional<GPS_MSG_S> pop_gps_msg();

#endif
