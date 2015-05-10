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

        // handle timestamps
        lastTimestamp_ = mavlinkMessages_->lastAttitude;		//needed to check if new message is available
        att->localTimestamp = mavlinkMessages_->lastAttitude;	//save lacalTimestamp (when message has been received by antenna)

		return true;
	}
	return false;
}


} /* namespace tracking */



