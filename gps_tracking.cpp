/*
 * VelBasedTracking.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */

#include <cmath>
#include <sstream>


#include <trackingsetup/gps_tracking.h>


namespace tracking {

GpsTrackingMode::GpsTrackingMode(TrackingEstimator* estimator):
		estimator_(estimator), panOffset_(0), tiltOffset_(0), magneticDeclination_(0), hardPositioningTilt_(false), hardPositioningPan_(false),
	azimuthAngle_(0), elevationAngle_(0), azimuthRate_(0), elevationRate_(0) {
	// TODO Auto-generated constructor stub

}

GpsTrackingMode::~GpsTrackingMode() {
	// TODO Auto-generated destructor stub
}

void GpsTrackingMode::setNorthOffset(float panAngleNorth) {
}

void GpsTrackingMode::setMagneticDeclination(float magneticDeclination) {
	magneticDeclination = magneticDeclination_;
}

void GpsTrackingMode::setMapping(float panOffset, float tiltOffset) {
	panOffset_ = panOffset;
	tiltOffset_ = tiltOffset;
}

void GpsTrackingMode::update(double curPanAngle, double curTiltAngle) {
	std::stringstream logmessage("Estimator based tracking | ");

	// read values from estimator
	azimuthAngle_ = estimator_->getAzimuth()-magneticDeclination_-panOffset_;
	elevationAngle_ = estimator_->getElevation();

	logmessage << "Estimated azimuth angle: " << azimuthAngle_ << ", elevation angle: " << elevationAngle_;
	addLogMessage(vl_DEBUG,logmessage.str());


	// Update setpoint
	// workaround to avoid overshoot of maxon controller
	if (fabs(azimuthAngle_ - curPanAngle)  > 30 || fabs(azimuthAngle_ - (curPanAngle+360.0)) > 30) {
		if (hardPositioningPan_) {
			setNewPanSetpoint(ct_undefined,0);
			addLogMessage(vl_DEBUG,"Estimator based tracking | Hard Positioning Pan");

		} else {
			hardPositioningPan_ = true;
			addLogMessage(vl_DEBUG,"Estimator based tracking | Hard Positioning Pan...");
			setNewPanSetpoint(ct_abspos,azimuthAngle_);
		}
	} else {
		if (hardPositioningPan_) {
			hardPositioningPan_ = false;
		}
		setNewPanSetpoint(ct_abspos,azimuthAngle_);
	}

	if (fabs(elevationAngle_ - curTiltAngle)  > 15) {
		if (hardPositioningTilt_) {
			setNewTiltSetpoint(ct_undefined,0);
			addLogMessage(vl_DEBUG,"Estimator based tracking | Hard Positioning Tilt");

		} else {
			hardPositioningTilt_ = true;
			addLogMessage(vl_DEBUG,"Estimator based tracking | Hard Positioning Tilt ...");
			setNewTiltSetpoint(ct_abspos,elevationAngle_);
		}
	} else {
		if (hardPositioningTilt_) {
			hardPositioningTilt_ = false;
		}
		setNewTiltSetpoint(ct_abspos,elevationAngle_);
	}

}

} /* namespace tracking */
