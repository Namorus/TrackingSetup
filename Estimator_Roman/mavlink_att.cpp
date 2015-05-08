/*
 * mavlink_att.cpp
 *
 *  Created on: May 8, 2015
 *      Author: Maverick
 */

#include <trackingsetup/mavlink_att.h>

namespace tracking {

MavlinkAtt::MavlinkAtt(MavlinkMessages* mavlinkMessages, std::string label) :
        mavlinkMessages_(mavlinkMessages), lastTimestamp_(0) {
	this->setLabel(label);
}

bool MavlinkAtt::getAtt(Att* att) {

    if (mavlinkMessages_->lastAttitude > lastTimestamp_) {
        att->roll = mavlinkMessages_->attitude.roll;
        lastTimestamp_ = mavlinkMessages_->lastAttitude;
		return true;
	}
	return false;
}

MavlinkAtt::~MavlinkAtt() {
	// TODO Auto-generated destructor stub
}

} /* namespace tracking */




