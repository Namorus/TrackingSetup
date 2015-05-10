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

	//void virtual KFupdateEstimate() = 0;

	// void virtual KFpredictEstimate() = 0; //---------ROMAN

	void setNewRemoteGPos(GlobalPos& remoteGpos);

	//-----------------------------------ROMAN--------Attitude
	void setNewAttitude(Attitude& attitude);

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

	friend std::ostream& operator<<(std::ostream& out, const TrackingEstimator& estimator);

protected:
	GPSPos antennaPos_;
	GlobalPos targetGlobalPos_;
	LocalPos targetPosLocal_;

	//-----------------------------------------------ROMAN-------------------------------------Attitude
	Attitude targetAttitude_;

	GPSPos targetEstimatedPos_;
	LocalPos targetEstimatedPosLocal_;

	LocalPos targetEstimatedVel_;

	GeographicLib::LocalCartesian antennaLocalCartesian_;

	//GeographicLib::Geocentric earth_;

};

} /* namespace tracking */

#endif /* TRACKINGESTIMATOR_H_ */
