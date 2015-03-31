/*
 * TrackingEstimator.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: Thomas
 */
#include <cmath>

#include "includes/trackingsetup/trackingEstimator.h"

namespace tracking {
TrackingEstimator::TrackingEstimator() {
	earth_ = GeographicLib::Geocentric::WGS84;
	antennaLocalCartesian_ = GeographicLib::LocalCartesian(earth_);
}

TrackingEstimator::~TrackingEstimator() {

}

void TrackingEstimator::setAntennaPos(GPSPos antennaPos) {
	antennaPos_ = antennaPos;
	antennaLocalCartesian_.Reset(antennaPos.lat,antennaPos.lon,antennaPos.alt);

}

void TrackingEstimator::setNewRemoteGPos(GlobalPos& remoteGpos) {
	targetGlobalPos_ = remoteGpos;
}


void TrackingEstimator::updateAntennaPos(GPSPos antennaPos) {
	setAntennaPos(antennaPos);
	updateEstimate();
}

double TrackingEstimator::getDistance() {
	return sqrt(targetEstimatedPosLocal_.x*targetEstimatedPosLocal_.x+targetEstimatedPosLocal_.y*targetEstimatedPosLocal_.y);
}

double TrackingEstimator::getLOSDistance() {
	return sqrt(targetEstimatedPosLocal_.x*targetEstimatedPosLocal_.x+targetEstimatedPosLocal_.y*targetEstimatedPosLocal_.y+targetEstimatedPosLocal_.z*targetEstimatedPosLocal_.z);;
}

double TrackingEstimator::getAzimuth() {
	return rad2deg(fmod(atan2(M_PI_2-targetEstimatedPosLocal_.y,targetEstimatedPosLocal_.x),2*M_PI));
}

double TrackingEstimator::getElevation() {
	double groundDistance = getDistance();
	return rad2deg(atan2(targetEstimatedPosLocal_.z,groundDistance));
}

double TrackingEstimator::deg2rad(double deg) {
	return M_PI / 180.0 * deg;
}

double TrackingEstimator::getGroundspeed() {

	return 0;
}

void TrackingEstimator::getGlobalPosEstimate(GPSPos& posEstimate) {
	posEstimate = targetGlobalPos_.position;
}

void TrackingEstimator::getLocalPosEstimate(LocalPos& posEstimate) {
	posEstimate = targetPosLocal_;
}

void TrackingEstimator::getLocalVelEstimate(LocalPos& velEstimate) {
	velEstimate = targetEstimatedVel_;
}

void TrackingEstimator::getEstimate(GlobalPos& estimate) {
	estimate = targetGlobalPos_;
}

double TrackingEstimator::rad2deg(double rad) {
	return 180.0 / M_PI * rad;
}
} /* namespace tracking */
