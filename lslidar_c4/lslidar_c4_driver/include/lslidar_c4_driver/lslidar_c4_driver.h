/***************************************************************************
Copyright 2018 The Leishen Authors. All Rights Reserved                     /
                                                                            /
Licensed under the Apache License, Version 2.0 (the "License");             /
you may not use this file except in compliance with the License.            /
You may obtain a copy of the License at                                     /
                                                                            /
    http://www.apache.org/licenses/LICENSE-2.0                              /
                                                                            /
Unless required by applicable law or agreed to in writing, software         /
distributed under the License is distributed on an "AS IS" BASIS,           /
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    /
See the License for the specific language governing permissions and         /
limitations under the License.                                              /
****************************************************************************/

#ifndef LSLIDAR_C4_DRIVER_H
#define LSLIDAR_C4_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include <time.h>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <lslidar_c4_msgs/LslidarC4Packet.h>
#include <lslidar_c4_msgs/LslidarC4ScanUnified.h>

namespace apollo {
namespace drivers {
namespace lslidar_c4_driver {

//static uint16_t UDP_PORT_NUMBER = 8080;
static uint16_t PACKET_SIZE = 1206;

class LslidarC4Driver {
public:

    LslidarC4Driver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarC4Driver();

    bool initialize();
    bool polling();

    void initTimeStamp(void);    
    void getFPGA_GPSTimeStamp(lslidar_c4_msgs::LslidarC4PacketPtr &packet);

    typedef boost::shared_ptr<LslidarC4Driver> LslidarC4DriverPtr;
    typedef boost::shared_ptr<const LslidarC4Driver> LslidarC4DriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_c4_msgs::LslidarC4PacketPtr& msg);

    // Ethernet relate variables
    std::string device_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
    int socket_id;
    int cnt_gps_ts;

    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string frame_id;
    ros::Publisher packet_pub;    

    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;

    uint64_t pointcloudTimeStamp;
    uint64_t GPSStableTS;
    uint64_t GPSCountingTS;
    uint64_t last_FPGA_ts;
    uint64_t GPS_ts;
    unsigned char packetTimeStamp[10];
    struct tm cur_time;
    unsigned short int us;
    unsigned short int ms;
    ros::Time timeStamp;
};

typedef LslidarC4Driver::LslidarC4DriverPtr LslidarC4DriverPtr;
typedef LslidarC4Driver::LslidarC4DriverConstPtr LslidarC4DriverConstPtr;

} // namespace lslidar_driver
}
}
#endif // _LSLIDAR_C4_DRIVER_H_
