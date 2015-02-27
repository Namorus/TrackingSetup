/*
 * TAGPSTracking.h
 *
 *  Created on: Jun 18, 2013
 *      Author: thomas
 */

#ifndef TAGPSTRACKING_H_
#define TAGPSTRACKING_H_

#include <trackingsetup/tracking_mode.h>

namespace tracking {

class TAGPSTracking: public tracking::TrackingMode {
public:
	TAGPSTracking();

	void setNorthOffset(float panAngleNorth);

	void setMagneticDeclination(float magneticDeclination_);

	void setMapping(float panOffset, float tiltOffset);

//	void setAntennaPos(GPSPos antennaPOS);

	void update(GPSPos& antennaPos, GPSPos& targetPos);

	static double getLOSdistance(GPSPos* posA, GPSPos* posB);

	static double getDistance(GPSPos* posA, GPSPos* posB);

	static double getAzimuth(GPSPos* posA, GPSPos* posB);

	static double getBearing(GPSPos* posA, GPSPos* posB);

	static double deg2rad(double deg);

	static double rad2deg(double rad);

private:
	float panOffset_, tiltOffset_;
	float magneticDeclination_;

	// GPSPos antennaPOS;
};

} /* namespace tracking */
#endif /* TAGPSTRACKING_H_ */
