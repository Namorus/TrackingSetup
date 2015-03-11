/*
 * TSMavlinkGPS.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: asl
 */
#include <sys/time.h>

#include <trackingsetup/mavlink_gpos.h>

namespace tracking {

MavlinkGpos::MavlinkGpos(MavlinkMessages* mavlinkMessages) :
        mavlinkMessages_(mavlinkMessages), lastPosTimestamp_(0), lastVelTimestamp_(0) {
}

bool MavlinkGpos::getPos(GPSPos* pos) {
    if (mavlinkMessages_->gps_raw_int.fix_type < 3) {
//		pos->elev = -1;
//		pos->lat = 99;
//		pos->lon = 199;
		return false;
	}

    if (mavlinkMessages_->lastGlobalPosition > lastPosTimestamp_) {
        pos->elev = mavlinkMessages_->global_position_int.alt / 1000.0;
        pos->lon = 1E-7 * mavlinkMessages_->global_position_int.lon;
        pos->lat = 1E-7 * mavlinkMessages_->global_position_int.lat;
        lastPosTimestamp_ = mavlinkMessages_->lastGlobalPosition;
		return true;
	}
	return false;
}

bool MavlinkGpos::getVel(GPSPos* vel) {
    if (mavlinkMessages_->gps_raw_int.fix_type < 3) {
		return false;
	}

    if (mavlinkMessages_->lastGlobalPosition > lastVelTimestamp_) {
    	vel->elev = 1E-2 * mavlinkMessages_->global_position_int.vz;
    	vel->lon = 1E-2 * mavlinkMessages_->global_position_int.vx;
    	vel->lat = 1E-2 * mavlinkMessages_->global_position_int.vy;
    	lastVelTimestamp_ = mavlinkMessages_->lastGlobalPosition;
		return true;
	}
	return false;
}

MavlinkGpos::~MavlinkGpos() {
	// TODO Auto-generated destructor stub
}

bool MavlinkGpos::getGpos(GlobalPos* gpos) {
	gpos->timestamp = mavlinkMessages_->global_position_int.time_boot_ms;

	timeval now;
	gettimeofday(&now,NULL);
	gpos->localTimestamp = now.tv_sec*1e6 + now.tv_usec;

	bool result = getPos(&gpos->position);
    result &= getVel(&gpos->speed);
    return result;
}

} /* namespace tracking */
