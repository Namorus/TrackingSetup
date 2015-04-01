/*
 * VelBasedTracking.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */

#include "trackingsetup/VelBasedTracking.h"

namespace tracking {

VelBasedGpsTrackingMode::VelBasedGpsTrackingMode(TrackingEstimator* estimator):
		estimator_(estimator), panOffset_(0), tiltOffset_(0), magneticDeclination_(0), hardPositioning_(true),
	azimuthAngle_(0), elevationAngle_(0), azimuthRate(0), elevationRate_(0) {
	// TODO Auto-generated constructor stub

}

VelBasedGpsTrackingMode::~VelBasedGpsTrackingMode() {
	// TODO Auto-generated destructor stub
}

void VelBasedGpsTrackingMode::setNorthOffset(float panAngleNorth) {
}

void VelBasedGpsTrackingMode::setMagneticDeclination(float magneticDeclination_) {
}

void VelBasedGpsTrackingMode::setMapping(float panOffset, float tiltOffset) {
}

void VelBasedGpsTrackingMode::update(double curPanAngle, double curTiltAngle) {
	// read values from estimator
	azimuthAngle_ = estimator_->getAzimuth();
	elevationAngle_ = estimator_->getElevation();

	estimator_->getLocalPosEstimate(localPos_);
	estimator_->getLocalVelEstimate(localVel_);

	if (fabs(azimuthAngle_ - curPanAngle)  > 45 || fabs(elevationAngle_ - curTiltAngle) > 30) {
		if (hardPositioning_) {
			setNewSetpoints(ct_undefined,0,0);
		} else {
			hardPositioning_ = true;
			setNewSetpoints(ct_abspos,azimuthAngle_-magneticDeclination_-panOffset_,elevationAngle_);
		}
		return;
	}

	// 1. Calculate pan and tilt speeds from pos & vel
	azimuthRate = -(localPos_.x*localVel_.y-localPos_.y*localVel_.x)/(localPos_.x*localPos_.x+localPos_.y*localPos_.y);
	double elevationRateNumerator = ((localPos_.x*localPos_.x+localPos_.y*localPos_.y)*localVel_.z-localPos_.z*(localPos_.x*localVel_.x+localPos_.y*localVel_.y));
	double elevationRateDenominator = (sqrt(localPos_.x*localPos_.x+localPos_.y*localPos_.y)*sqrt(localPos_.x*localPos_.x+localPos_.y*localPos_.y+localPos_.z*localPos_.z));
	elevationRate_ = elevationRateNumerator / elevationRateDenominator;


	// 2. correction of speed based on measured angle
	double newPanSpeed = azimuthRate - (azimuthAngle_ - curPanAngle);
	double newTiltSpeed = elevationRate_ - (elevationAngle_ - curTiltAngle);

	// 3. Update setpoint
	setNewSetpoints(ct_velocity, newPanSpeed, newTiltSpeed);
}

} /* namespace tracking */
