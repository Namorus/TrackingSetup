/*
 * TSmavlinkMag.cpp
 *
 *  Created on: Feb 4, 2015
 *      Author: asl
 */

#include <trackingsetup/mavlink_mag.h>

namespace tracking {

TSmavlinkMag::TSmavlinkMag(MavlinkMessages* mavlinkMessages) :
    mavlinkMessages_(mavlinkMessages),
	lastTimestamp_(0) {

}

bool TSmavlinkMag::getMag(MagReading* mag) {
    if (mavlinkMessages_->lastHighresImu > lastTimestamp_) {
        mag->magX = mavlinkMessages_->highres_imu.xmag;
        mag->magY = mavlinkMessages_->highres_imu.ymag;
        mag->magZ = mavlinkMessages_->highres_imu.zmag;
        lastTimestamp_ = mavlinkMessages_->lastHighresImu;
		return true;
	} else {
		return false;
	}
}

TSmavlinkMag::~TSmavlinkMag() {
	// TODO Auto-generated destructor stub
}

} /* namespace tracking */
