/*
 * VelBasedTracking.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */

#include "trackingsetup/VelBasedTracking.h"

namespace tracking {

VelBasedGpsTrackingMode::VelBasedGpsTrackingMode(TrackingEstimator* estimator):
	panOffset_(0), tiltOffset_(0), magneticDeclination_(0) {
	// TODO Auto-generated constructor stub

}

VelBasedGpsTrackingMode::~VelBasedGpsTrackingMode() {
	// TODO Auto-generated destructor stub
}

void VelBasedGpsTrackingMode::setNorthOffset(float panAngleNorth) {
}

void VelBasedGpsTrackingMode::setMagneticDeclination(
		float magneticDeclination_) {
}

void VelBasedGpsTrackingMode::setMapping(float panOffset, float tiltOffset) {
}

void VelBasedGpsTrackingMode::update() {

	// 1. Calculate pan and tilt speeds from pos & vel

	// 2. correction of speed based on measured angle

	// 3. Update setpoint
	// setNewSetpoints(ct_abspos, bearing - panOffset_ - magneticDeclination_, azimuth);
}

} /* namespace tracking */
