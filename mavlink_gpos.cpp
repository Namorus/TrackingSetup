/*
 * TSMavlinkGPS.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: asl
 */

#include <trackingsetup/mavlink_gpos.h>

namespace tracking {

MavlinkGpos::MavlinkGpos(MavlinkMessages* mavlinkMessages, std::string label) :
        mavlinkMessages_(mavlinkMessages), lastTimestamp_(0) {
	this->setLabel(label);
}

bool MavlinkGpos::getPos(GPSPos* pos) {
    if (mavlinkMessages_->gps_raw_int.fix_type < 3) {
//		pos->elev = -1;
//		pos->lat = 99;
//		pos->lon = 199;
		return false;
	}

    if (mavlinkMessages_->lastGlobalPosition > lastTimestamp_) {
        pos->elev = mavlinkMessages_->global_position_int.alt / 1000.0;
        pos->lon = 1E-7 * mavlinkMessages_->global_position_int.lon;
        pos->lat = 1E-7 * mavlinkMessages_->global_position_int.lat;
        lastTimestamp_ = mavlinkMessages_->lastGlobalPosition;
		return true;
	}
	return false;
}

float MavlinkGpos::getPosAccuracy() {
	double posAccuracy = 100;

	return posAccuracy;
}

MavlinkGps::~MavlinkGps() {
	// TODO Auto-generated destructor stub
}

} /* namespace tracking */
