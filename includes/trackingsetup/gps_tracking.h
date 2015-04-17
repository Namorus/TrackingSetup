/*
 * VelBasedTracking.h
 *
 *  Created on: Mar 31, 2015
 *      Author: thomas
 */

#ifndef VELBASEDTRACKING_H_
#define VELBASEDTRACKING_H_

#include <trackingsetup/trackingEstimator.h>
#include <trackingsetup/tracking_mode.h>

namespace tracking {

class GpsTrackingMode: public tracking::TrackingMode {
public:
	GpsTrackingMode(TrackingEstimator* estimator);
	virtual ~GpsTrackingMode();

	void setNorthOffset(float panAngleNorth);

	void setMagneticDeclination(float magneticDeclination);

	void setMapping(float panOffset, float tiltOffset);

	void update(double curPanAngle, double curTiltAngle);

private:

	TrackingEstimator* estimator_;

	float panOffset_, tiltOffset_;
	float magneticDeclination_;

	bool hardPositioningTilt_;
	bool hardPositioningPan_;

	double azimuthAngle_, elevationAngle_;

	LocalPos targetLocalPos_,targetLocalVel_;

	double azimuthRate_, elevationRate_;


};

} /* namespace tracking */

#endif /* VELBASEDTRACKING_H_ */
