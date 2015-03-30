/*
 * TrackingEstimator.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: asl
 */

#include "TrackingEstimator.h"

namespace tracking {

void TrackingEstimator::setAntennaPos(GPSPos antennaPos) {
	antennaPos_ = antennaPos;
	antennaLocalCartesian_.Reset(antennaPos.lat,antennaPos.lon,antennaPos.elev);

}

void TrackingEstimator::updateAntennaPos(GPSPos antennaPos) {
	setAntennaPos(antennaPos);
	updateEstimate();
}

double TrackingEstimator::getDistance() {
	return sqrt(targetEstimatedPosLocal_[0]*targetEstimatedPosLocal_[0]+targetEstimatedPosLocal_[1]*targetEstimatedPosLocal_[1]);
}

double TrackingEstimator::getLOSDistance() {
	return sqrt(targetEstimatedPosLocal_[0]*targetEstimatedPosLocal_[0]+targetEstimatedPosLocal_[1]*targetEstimatedPosLocal_[1]+targetEstimatedPosLocal_[2]*targetEstimatedPosLocal_[2]);;
}

double TrackingEstimator::getAzimuth() {
	return rad2deg(fmod(atan2(targetEstimatedPosLocal_[1],targetEstimatedPosLocal_[0]),2*M_PI));
}

double TrackingEstimator::getElevation() {
	double groundDistance = getDistance();
	return rad2deg(atan2(targetEstimatedPosLocal_[2],groundDistance));
}

double TrackingEstimator::deg2rad(double deg) {
	return M_PI / 180.0 * deg;
}

double TrackingEstimator::getGroundspeed() {

	return 0;
}

double TrackingEstimator::rad2deg(double rad) {
	return 180.0 / M_PI * rad;
}
} /* namespace tracking */
