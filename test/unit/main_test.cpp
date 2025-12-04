#include <gtest/gtest.h>
#include "gps_parser.h"

extern "C" {
extern void parse_gps_message(char* message);
extern GPS_MSG_S g_gps_curr_msg;
}

TEST(GnssTest, parse_gps_GNGGA) {
    char gngga_msg[] = "$GNGGA,024950.000,3113.08495,N,12130.88516,E,1,24,0.8,13.0,M,11.4,M,,*73\r\n";
    parse_gps_message(gngga_msg);
    EXPECT_EQ(g_gps_curr_msg.gngga.lat_ang, 31.2180825);
    EXPECT_EQ(g_gps_curr_msg.gngga.lon_ang, 121.5147526666666667);
    EXPECT_EQ(g_gps_curr_msg.gngga.msl, 13.0);
}

TEST(GnssTest, parse_gps_GNVTG) {
    char gnvtg_msg[] = "$GNVTG,60.29,T,,M,2.85,N,5.28,K,A*1E\r\n";
    parse_gps_message(gnvtg_msg);
    EXPECT_EQ(g_gps_curr_msg.gnvtg.kph, 5.28);
    EXPECT_EQ(g_gps_curr_msg.gnvtg.cogt_ang, 60.29);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
