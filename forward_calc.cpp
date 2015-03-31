/*
 * forward_calc.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */
#include <sys/time.h>

#include <trackingsetup/forward_calc.h>

namespace tracking {

ForwardCalc::ForwardCalc() {
	// TODO Auto-generated constructor stub

}

ForwardCalc::~ForwardCalc() {
	// TODO Auto-generated destructor stub
}

void ForwardCalc::updateEstimate() {
	// calculate time since last GPOS update
	timeval now;
	gettimeofday(&now,NULL);
	uint64_t currentTimestamp = now.tv_sec*1e6 + now.tv_usec;
	uint32_t dT = currentTimestamp - targetGlobalPos_.localTimestamp;

	// update based on ground speed
	targetEstimatedPosLocal_.x = targetPosLocal_.x + targetGlobalPos_.velocity.lat*dT*1e-6;
	targetEstimatedPosLocal_.y = targetPosLocal_.y + targetGlobalPos_.velocity.lon*dT*1e-6;
	targetEstimatedPosLocal_.z = targetPosLocal_.z + targetGlobalPos_.velocity.elev*dT*1e-6;

	// convert back to WGS84
	antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,targetEstimatedPosLocal_.y,targetEstimatedPosLocal_.z,targetEstimatedPos_.lat,targetEstimatedPos_.lon,targetEstimatedPos_.elev);

	// velocity estimate is previous groundspeed measurement
	targetEstimatedVel_ = targetGlobalPos_.velocity;
}

} /* namespace tracking */
