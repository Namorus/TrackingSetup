/*
 * mavlink_attitude.cpp
 *
 *  Created on: May 8, 2015
 *      Author: Maverick
 */

#include <sys/time.h>

#include <trackingsetup/mavlink_attitude.h>

namespace tracking {

MavlinkAttitude::MavlinkAttitude(MavlinkMessages* mavlinkMessages) :
        mavlinkMessages_(mavlinkMessages), lastAttTimestamp_(0) {
}

bool MavlinkAttitude::getAtt(Att* att) {

    if (mavlinkMessages_->lastAttitude > lastAttTimestamp_) {
        att->roll = mavlinkMessages_->attitude.roll;
        lastAttTimestamp_ = mavlinkMessages_->lastAttitude;
		return true;
	}
	return false;
}


MavlinkAttitude::~MavlinkAttitude() {
	// TODO Auto-generated destructor stub
}

bool MavlinkAttitude::getAttitude(Attitude* attitude) {
	attitude->timestamp = mavlinkMessages_->attitude.time_boot_ms;

	timeval now;
	gettimeofday(&now,NULL);
	attitude->localTimestamp = now.tv_sec*1e6 + now.tv_usec;

	bool result = getAtt(&attitude->attitude);
    return result;
}

} /* namespace tracking */



