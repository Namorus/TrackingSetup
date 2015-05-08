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
        mavlinkMessages_(mavlinkMessages), lastTimestamp_(0) {
}

MavlinkAttitude::~MavlinkAttitude() {
	// TODO Auto-generated destructor stub
}

bool MavlinkAttitude::getAttitude(Attitude* att) {

    if (mavlinkMessages_->lastAttitude > lastTimestamp_) {
        att->roll = mavlinkMessages_->attitude.roll;
        att->pitch = mavlinkMessages_->attitude.pitch;
        att->yaw = mavlinkMessages_->attitude.yaw;

        att->rollrate = mavlinkMessages_->attitude.rollspeed;
        att->pitchrate = mavlinkMessages_->attitude.pitchspeed;
        att->yawrate = mavlinkMessages_->attitude.yawspeed;

        att->timestamp = mavlinkMessages_->attitude.time_boot_ms;
        lastTimestamp_ = mavlinkMessages_->lastAttitude; //???????

    	timeval now;
    	gettimeofday(&now,NULL);
    	att->localTimestamp = now.tv_sec*1e6 + now.tv_usec;

		return true;
	}
	return false;
}

/*
bool MavlinkAttitude::getAttitude(Attitude* attitude) {
	attitude->timestamp = mavlinkMessages_->attitude.time_boot_ms;

	timeval now;
	gettimeofday(&now,NULL);
	attitude->localTimestamp = now.tv_sec*1e6 + now.tv_usec;

	bool result = getAtt(&attitude->attitude);
    return result;
}
*/


} /* namespace tracking */



