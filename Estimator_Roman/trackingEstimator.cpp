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
	GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),GeographicLib::Constants::WGS84_f());
	antennaLocalCartesian_ = GeographicLib::LocalCartesian(earth);
}

TrackingEstimator::~TrackingEstimator() {

}

void TrackingEstimator::setAntennaPos(GPSPos antennaPos) {
	antennaPos_ = antennaPos;
	antennaLocalCartesian_.Reset(antennaPos.lat,antennaPos.lon,antennaPos.alt);

}

void TrackingEstimator::setNewRemoteGPos(GlobalPos& remoteGpos) {
	targetGlobalPos_ = remoteGpos;
	antennaLocalCartesian_.Forward(remoteGpos.position.lat,remoteGpos.position.lon,remoteGpos.position.alt,targetPosLocal_.x,targetPosLocal_.y,targetPosLocal_.z);

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
	return rad2deg(fmod(M_PI_2-atan2(targetEstimatedPosLocal_.y,targetEstimatedPosLocal_.x),2*M_PI));
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
	posEstimate = targetEstimatedPosLocal_;
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

std::ostream& operator<<(std::ostream& out,
		const TrackingEstimator& estimator) {
	out.precision(6);
	out << estimator.targetPosLocal_ << " "
		<< estimator.targetEstimatedPosLocal_ << " "
		<< estimator.targetEstimatedPos_;
	return out;
}

} /* namespace tracking */
