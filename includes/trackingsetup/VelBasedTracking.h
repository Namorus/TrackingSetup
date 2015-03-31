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

class VelBasedGpsTrackingMode: public tracking::TrackingMode {
public:
	VelBasedGpsTrackingMode(TrackingEstimator* estimator);
	virtual ~VelBasedGpsTrackingMode();

	void setNorthOffset(float panAngleNorth);

	void setMagneticDeclination(float magneticDeclination_);

	void setMapping(float panOffset, float tiltOffset);

	void update();

private:
	float panOffset_, tiltOffset_;
	float magneticDeclination_;

};

} /* namespace tracking */

#endif /* VELBASEDTRACKING_H_ */
