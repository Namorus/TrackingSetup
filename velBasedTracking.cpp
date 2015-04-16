/*
 * VelBasedTracking.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */
#include <trackingsetup/velBasedTracking.h>
#include <sstream>


namespace tracking {

VelBasedGpsTrackingMode::VelBasedGpsTrackingMode(TrackingEstimator* estimator):
		estimator_(estimator), panOffset_(0), tiltOffset_(0), magneticDeclination_(0), hardPositioning_(false),
	azimuthAngle_(0), elevationAngle_(0), azimuthRate(0), elevationRate_(0) {
	// TODO Auto-generated constructor stub

}

VelBasedGpsTrackingMode::~VelBasedGpsTrackingMode() {
	// TODO Auto-generated destructor stub
}

void VelBasedGpsTrackingMode::setNorthOffset(float panAngleNorth) {
}

void VelBasedGpsTrackingMode::setMagneticDeclination(float magneticDeclination) {
	magneticDeclination = magneticDeclination_;
}

void VelBasedGpsTrackingMode::setMapping(float panOffset, float tiltOffset) {
	panOffset_ = panOffset;
	tiltOffset_ = tiltOffset;
}

void VelBasedGpsTrackingMode::update(double curPanAngle, double curTiltAngle) {
	// read values from estimator
	azimuthAngle_ = estimator_->getAzimuth()-magneticDeclination_-panOffset_;
	elevationAngle_ = estimator_->getElevation();

	estimator_->getLocalPosEstimate(targetLocalPos_);
	estimator_->getLocalVelEstimate(targetLocalVel_);

	if (fabs(azimuthAngle_ - curPanAngle)  > 45 || fabs(elevationAngle_ - curTiltAngle) > 30) {
		if (hardPositioning_) {
			setNewSetpoints(ct_undefined,0,0);
		} else {
			hardPositioning_ = true;
			addLogMessage(vl_INFO,"Velocity based GPS tracking | Hard Positioning ...");
			setNewSetpoints(ct_abspos,azimuthAngle_,elevationAngle_);
		}
		return;
	}

	// 1. Calculate pan and tilt speeds from pos & vel
	azimuthRate = -(targetLocalPos_.x*targetLocalVel_.y-targetLocalPos_.y*targetLocalVel_.x)/(targetLocalPos_.x*targetLocalPos_.x+targetLocalPos_.y*targetLocalPos_.y);
	double elevationRateNumerator = ((targetLocalPos_.x*targetLocalPos_.x+targetLocalPos_.y*targetLocalPos_.y)*targetLocalVel_.z-targetLocalPos_.z*(targetLocalPos_.x*targetLocalVel_.x+targetLocalPos_.y*targetLocalVel_.y));
	double elevationRateDenominator = (sqrt(targetLocalPos_.x*targetLocalPos_.x+targetLocalPos_.y*targetLocalPos_.y)*sqrt(targetLocalPos_.x*targetLocalPos_.x+targetLocalPos_.y*targetLocalPos_.y+targetLocalPos_.z*targetLocalPos_.z));
	if (elevationRateDenominator > 0) {
		elevationRate_ = elevationRateNumerator / elevationRateDenominator;
	} else {
		elevationRate_ = 0;
	}


	// 2. correction of speed based on measured angle
	double newPanSpeed = estimator_->rad2deg(azimuthRate) - 0.5*(azimuthAngle_ - curPanAngle); // TODO: make gain configurable
	double newTiltSpeed = estimator_->rad2deg(elevationRate_) - 0.5*(elevationAngle_ - curTiltAngle); // TODO: make gain configurable

	// 3. Update setpoint
	std::stringstream logmessage("Velocity based GPS tracking | ");

	if (isnan(newPanSpeed)) {
		logmessage << "New pan speed is nan -> stopping";
		setNewSetpoints(ct_velocity, 0, 0);
	} else if (isnan(newTiltSpeed)) {
		logmessage << "New pan tilt is nan -> stopping";
		setNewSetpoints(ct_velocity, 0, 0);
	} else {
		logmessage << "New velocity setpoint (" << newPanSpeed << "/" << newTiltSpeed  << ")";
		setNewSetpoints(ct_velocity, newPanSpeed, newTiltSpeed);
	}

	addLogMessage(vl_INFO,logmessage.str());
}

} /* namespace tracking */
