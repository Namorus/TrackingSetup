/*
 * TSmavlinkMag.cpp
 *
 *  Created on: Feb 4, 2015
 *      Author: asl
 */

#include <trackingsetup/mavlink_mag.h>

namespace tracking {

MavlinkMagnetometer::MavlinkMagnetometer(MavlinkMessages* mavlinkMessages) :
    mavlinkMessages_(mavlinkMessages),
	lastTimestamp_(0) {

}

bool MavlinkMagnetometer::getMag(MagReading* mag) {
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

MavlinkMagnetometer::~MavlinkMagnetometer() {
	// TODO Auto-generated destructor stub
}

} /* namespace tracking */
