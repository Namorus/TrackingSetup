/*
 * TAGPSTracking.h
 *
 *  Created on: Jun 18, 2013
 *      Author: thomas
 */

#ifndef TAGPSTRACKING_H_
#define TAGPSTRACKING_H_

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <trackingsetup/tracking_mode.h>

namespace tracking {

class GpsTrackingMode: public tracking::TrackingMode {
public:
	GpsTrackingMode();

	void setNorthOffset(float panAngleNorth);

	void setMagneticDeclination(float magneticDeclination_);

	void setMapping(float panOffset, float tiltOffset);

	void setAntennaPos(GPSPos antennaPOS);

	void updateGPS(GPSPos& targetPos);

	void updateGPOS(GlobalPos& targetGlobalPos);

	void updateEstimated();

	static double getLOSdistance(GPSPos* posA, GPSPos* posB);

	static double getDistance(GPSPos* posA, GPSPos* posB);

	static double getAzimuth(GPSPos* posA, GPSPos* posB);

	static double getBearing(GPSPos* posA, GPSPos* posB);

	static double deg2rad(double deg);

	static double rad2deg(double rad);

private:
	float panOffset_, tiltOffset_;
	float magneticDeclination_;

	GPSPos antennaPos_;
	GlobalPos targetGlobalPos_;
	double targetPosLocal_[3];

	GPSPos targetEstimatedPos_;
	double targetEstimatedPosLocal_[3];

	GeographicLib::LocalCartesian antennaLocalCartesian_;

	GeographicLib::Geocentric earth_;

};

} /* namespace tracking */
#endif /* TAGPSTRACKING_H_ */
