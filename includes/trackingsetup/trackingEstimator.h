/*
 * TrackingEstimator.h
 *
 *  Created on: Mar 30, 2015
 *      Author: asl
 */

#ifndef TRACKINGESTIMATOR_H_
#define TRACKINGESTIMATOR_H_

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <trackingsetup/trackingsetup.h>

namespace tracking {

class TrackingEstimator: public TrackingSetup {
public:
	TrackingEstimator();
	virtual ~TrackingEstimator();

	void virtual updateEstimate() = 0;

	void getGlobalPosEstimate(GPSPos& posEstimate);

	void getLocalPosEstimate(LocalPos& posEstimate);

	void getLocalVelEstimate(LocalPos& velEstimate);

	void getEstimate(GlobalPos& estimate);

	void setAntennaPos(GPSPos antennaPOS);
	void updateAntennaPos(GPSPos antennaPOS);

	/*!
	 * Returns distance in ground plane based on current estimate
	 * @return Distance in ground plane
	 */
	double getDistance();

	/*!
	 * returns distance from antenna to tracked object based on current estimate
	 * @return line of sight distance to tracked object
	 */
	double getLOSDistance();

	/*!
	 * get azimuth angle of tracked object based on current estimate
	 * @return azimuth angle in degrees from north (clockwise)
	 */
	double getAzimuth();

	/*!
	 * get elevation angle of tracked object based on current estimate
	 * @return elevation angle in degrees from horizon upwards
	 */
	double getElevation();

	double getGroundspeed();

	static double deg2rad(double deg);

	static double rad2deg(double rad);

//	/*!
//	 * get line of sight distance in meters of two gps positions
//	 * @param posA GPSPos of A
//	 * @param posB GPSPos of B
//	 * @return norm(r_A - r_B)
//	 */
//	static double getLOSdistance(GPSPos* posA, GPSPos* posB);
//
//	/*!
//	 * get line of sight distance in meters of two gps positions
//	 * @param posA GPSPos of A
//	 * @param posB GPSPos of B
//	 * @return norm(r_A - r_B)
//	 */
//	static double getDistance(GPSPos* posA, GPSPos* posB);
//
//	/*!
//	 * get azimuth angle (pan) of posA to posB (clockwise from north)
//	 * @param posA GPSPos of A
//	 * @param posB GPSPos of B
//	 * @return
//	 */
//	static double getAzimuth(GPSPos* posA, GPSPos* posB);
//
//	/*!
//	 * get elevation angle (tilt) of posA to posB (measured from the horizon)
//	 * @param posA GPSPos of A
//	 * @param posB GPSPos of B
//	 * @return
//	 */
//	static double getElevation(GPSPos* posA, GPSPos* posB);

protected:
	GPSPos antennaPos_;
	GlobalPos targetGlobalPos_;
	LocalPos targetPosLocal_;

	GPSPos targetEstimatedPos_;
	LocalPos targetEstimatedPosLocal_;

	LocalPos targetEstimatedVel_;

	GeographicLib::LocalCartesian antennaLocalCartesian_;

	GeographicLib::Geocentric earth_;

};

} /* namespace tracking */

#endif /* TRACKINGESTIMATOR_H_ */
