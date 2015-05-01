/*
 * forward_calc.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */
#include <sys/time.h>
#include <sstream>

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

	// velocity estimate is previous groundspeed measurement
	// transform from NED to ENU
	targetEstimatedVel_.x = targetGlobalPos_.velocity.lon;
	targetEstimatedVel_.y = targetGlobalPos_.velocity.lat;
	targetEstimatedVel_.z = -targetGlobalPos_.velocity.alt;

	// update based on ground speed
	targetEstimatedPosLocal_.x = targetPosLocal_.x + targetEstimatedVel_.x*dT*1e-6;
	targetEstimatedPosLocal_.y = targetPosLocal_.y + targetEstimatedVel_.y*dT*1e-6;
	targetEstimatedPosLocal_.z = targetPosLocal_.z + targetEstimatedVel_.z*dT*1e-6;

	// convert back to WGS84
	antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,targetEstimatedPosLocal_.y,targetEstimatedPosLocal_.z,targetEstimatedPos_.lat,targetEstimatedPos_.lon,targetEstimatedPos_.alt);

	std::stringstream logmessage;
	logmessage << "Estimated local pos: " << targetEstimatedPosLocal_ << ", local vel: " << targetEstimatedVel_;
	addLogMessage(vl_DEBUG,logmessage.str());
}

} /* namespace tracking */
