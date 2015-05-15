//============================================================================
// Name        : mavlink_sender.cpp
// Author      : Thomas Mantel
// Version     :
// Copyright   : 
// Description : Hello World in C, Ansi-style
//============================================================================

#include <fstream>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <pthread.h>

#include <common/mavlink.h>
#include "mavlinkWriter.h"

using namespace mavlinkSender;

unsigned long long getusecs() {
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

int main(int argc, char* argv[]) {
    time_t start;
    time(&start);

    std::string label = "Port A on FT4232";
    int interface = 2;
    int vid = 0x0403;
    int pid = 0x6011;

	int baudrate = 57600;

	mavlinkWriter writer = mavlinkWriter(vid,pid,interface,baudrate,label);
    if(!writer.openSerial()) {
    	return(-1);
    }

    writer.startHeartbeatThread();

    mavlink_message_t msg;

    mavlink_gps_raw_int_t gpsMsg;
    gpsMsg.time_usec = getusecs();
    gpsMsg.lat = 0;
    gpsMsg.lon = 0;
    gpsMsg.alt = (int) 500000;
    gpsMsg.cog = 0;
    gpsMsg.vel = 0;
    gpsMsg.eph = 100;
    gpsMsg.epv = 100;
    gpsMsg.fix_type = 3;
    gpsMsg.satellites_visible = 11;

    mavlink_global_position_int_t gposMsg;
    gposMsg.lat = 0;
    gposMsg.lon = 0;
    gposMsg.alt = 500000;
    gposMsg.hdg = 0;
    gposMsg.relative_alt = 0;
    gposMsg.time_boot_ms = 0;
    gposMsg.vx = 0;
    gposMsg.vy = 0;
    gposMsg.vz = 0;

    mavlink_attitude_t attitudeMsg;
    attitudeMsg.roll = 0;
    attitudeMsg.pitch = 0;
    attitudeMsg.yaw = 0;
    attitudeMsg.rollspeed = 0;
    attitudeMsg.pitchspeed = 0;
    attitudeMsg.yawspeed = 0;
    attitudeMsg.time_boot_ms = 0;


    // open file
    std::ifstream trackfs;

    trackfs.open ("trajectory.csv", std::ifstream::in);

    float ti;
    bool sendPos, sendAtt;
    double lat, lon;
    float alt, vx, vy, vz;
    float roll, pitch, yaw, rollrate, pitchrate, yawrate;


    while (!trackfs.eof()) {
        sendPos = false;
        sendAtt = false;

    	trackfs >> ti;
        trackfs.ignore(1,';');

        trackfs >> sendPos;
        trackfs.ignore(1,';');
        trackfs >> sendAtt;
        trackfs.ignore(1,';');

        trackfs >> lat;
        trackfs.ignore(1,';');
        trackfs >> lon;
        trackfs.ignore(1,';');
        trackfs >> alt;
        trackfs.ignore(1,';');

        trackfs >> vx;
        trackfs.ignore(1,';');
        trackfs >> vy;
        trackfs.ignore(1,';');
        trackfs >> vz;
        trackfs.ignore(1,';');

        trackfs >> roll;
        trackfs.ignore(1,';');
        trackfs >> pitch;
        trackfs.ignore(1,';');
        trackfs >> yaw;
        trackfs.ignore(1,';');

        trackfs >> rollrate;
        trackfs.ignore(1,';');
        trackfs >> pitchrate;
        trackfs.ignore(1,';');
        trackfs >> yawrate;
        trackfs.ignore(1,';');

//        float lat, lon;
//        std::cin >> lat >> lon;

        gposMsg.time_boot_ms = (int) difftime(time(NULL),start);
        gposMsg.lat = (int) (lat*1e7);
        gposMsg.lon = (int) (lon*1e7);
        gposMsg.alt = (int) (alt*1e3);
        gposMsg.vx = (int) (vx*1e2);
        gposMsg.vy = (int) (vy*1e2);
        gposMsg.vz = (int) (vz*1e2);

        gpsMsg.time_usec = getusecs();
        gpsMsg.lat = gposMsg.lat;
        gpsMsg.lon = gposMsg.lon;
        gpsMsg.alt = gposMsg.alt;

        attitudeMsg.roll = roll;
        attitudeMsg.pitch = pitch;
        attitudeMsg.yaw = yaw;
        attitudeMsg.rollspeed = rollrate;
        attitudeMsg.pitchspeed = pitchrate;
        attitudeMsg.yawspeed = yawrate;
        attitudeMsg.time_boot_ms = (int) difftime(time(NULL),start);

        while(ti >= difftime(time(NULL),start)) {
        	usleep(10000);//1000
        }
        if (sendPos) {
			std::cout << std::endl;
			std::cout << ti << ": (" << lat << "N " << lon << "E @ " << alt << "m ) - " << "(" << vx << "/" << vy << "/" << vz << ")" << std::endl;

			mavlink_msg_global_position_int_encode(1,255,&msg,&gposMsg);
			if(!writer.sendMessage(msg)) {
				std::cout << "error sending global position" << std::endl;
			} else {
				// std::cout << "(" << lat/1e7 << "/" << lon/1e7 << ")" << std::endl;
			}
			mavlink_msg_gps_raw_int_encode_chan(1,255,MAVLINK_COMM_1,&msg,&gpsMsg);
			writer.sendMessage(msg);
        }

        if (sendAtt) {
        	std::cout << ti << ": ( " << roll/M_PI*180.0 << "° / " << pitch/M_PI*180.0 << "° / " << yaw/M_PI*180.0 << "° ) - " << "(" << rollrate/M_PI*180.0 << "°/s / " << pitchrate/M_PI*180.0 << "°/s / " << yawrate/M_PI*180.0 << "°/s)" << std::endl;
        	mavlink_msg_attitude_encode(1,255,&msg,&attitudeMsg);
        	if(!writer.sendMessage(msg)) {
        		std::cout << "error sending attitude" << std::endl;
        	}
        }

    }

    writer.stopSending();

	writer.closeSerial();

    std::cout << "Bytes written: " << writer.getBytesWritten() << std::endl;



	return EXIT_SUCCESS;
}
