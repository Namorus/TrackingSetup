/*
 * TAGPSTracking.cpp
 *
 *  Created on: Jun 18, 2013
 *      Author: thomas
 */
#include <sys/time.h>

#include <trackingsetup/gps_tracking.h>

namespace tracking {

GpsTrackingMode::GpsTrackingMode() :
		panOffset_(0), tiltOffset_(0), magneticDeclination_(0) {
	earth_ = GeographicLib::Geocentric::WGS84;
	antennaLocalCartesian_ = GeographicLib::LocalCartesian(earth_);
	targetEstimatedPosLocal_[0] = 0;
	targetEstimatedPosLocal_[1] = 0;
	targetEstimatedPosLocal_[2] = 0;

	targetPosLocal_[0] = 0;
	targetPosLocal_[1] = 0;
	targetPosLocal_[2] = 0;
}

void GpsTrackingMode::setMapping(float panOffset, float tiltOffset) {
	panOffset_ = panOffset;
	tiltOffset_ = tiltOffset;
}

void GpsTrackingMode::setMagneticDeclination(float magneticDeclination) {
	magneticDeclination_ = magneticDeclination;
}


void GpsTrackingMode::setAntennaPos(GPSPos antennaPos) {
	antennaPos_ = antennaPos;
	antennaLocalCartesian_.Reset(antennaPos.lat,antennaPos.lon,antennaPos.alt);

}

void GpsTrackingMode::updateGPS(GPSPos& targetPos) {
	double bearing = getBearing(&antennaPos_, &targetPos);
	double azimuth = getAzimuth(&antennaPos_, &targetPos);

	setNewSetpoints(ct_abspos, bearing - panOffset_ - magneticDeclination_, azimuth);
}

void GpsTrackingMode::updateGPOS(GlobalPos& targetGlobalPos) {
	targetGlobalPos_ = targetGlobalPos;

	// calculate target position in local frame
	antennaLocalCartesian_.Forward(targetGlobalPos_.position.lat,targetGlobalPos_.position.lon,targetGlobalPos_.position.alt,targetPosLocal_[0],targetPosLocal_[1],targetPosLocal_[2]);

	// received position is used as estimate
	//TODO: maybe add some look ahead here
	targetEstimatedPosLocal_[0] = targetPosLocal_[0];
	targetEstimatedPosLocal_[1] = targetPosLocal_[1];
	targetEstimatedPosLocal_[2] = targetPosLocal_[2];
	targetEstimatedPos_ = targetGlobalPos.position;


	double bearing = getBearing(&antennaPos_, &targetEstimatedPos_);
//	double azimuth = getAzimuth(&antennaPos_, &targetEstimatedPos_);
	double azimuth = getAzimuthFromLocal();

//	std::cout << "New projected distance: " << getDistance() << ", new azimuth: " << azimuth << std::endl;

	setNewSetpoints(ct_abspos, bearing - panOffset_ - magneticDeclination_, azimuth);
}

void GpsTrackingMode::updateEstimated() {
//	antennaLocalCartesian_.Forward(targetGlobalPos_.position.lat,targetGlobalPos_.position.lon,targetGlobalPos_.position.elev,targetEstimatedPosLocal_[0],targetEstimatedPosLocal_[1],targetEstimatedPosLocal_[2]);

	// calculate time since last GPOS update
	timeval now;
	gettimeofday(&now,NULL);
	uint64_t currentTimestamp = now.tv_sec*1e6 + now.tv_usec;
	uint32_t dT = currentTimestamp - targetGlobalPos_.localTimestamp;

	// update based on ground speed
	targetEstimatedPosLocal_[0] = targetPosLocal_[0] + targetGlobalPos_.velocity.lon*dT*1e-6;
	targetEstimatedPosLocal_[1] = targetPosLocal_[1] + targetGlobalPos_.velocity.lat*dT*1e-6;
	targetEstimatedPosLocal_[2] = targetPosLocal_[2] - targetGlobalPos_.velocity.alt*dT*1e-6;

	// std::cout << "Target in local frame coordinates: (" << targetEstimatedPosLocal_[0] << ", " << targetEstimatedPosLocal_[1] << ", " << targetEstimatedPosLocal_[2] << "), dT since last GPOS: " << dT << std::endl;

	// convert back to WGS84
	antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_[0],targetEstimatedPosLocal_[1],targetEstimatedPosLocal_[2],targetEstimatedPos_.lat,targetEstimatedPos_.lon,targetEstimatedPos_.alt);


	double bearing = getBearing(&antennaPos_, &targetEstimatedPos_);
//	double azimuth = getAzimuth(&antennaPos_, &targetEstimatedPos_);
	double azimuth = getAzimuthFromLocal();

//	std::cout << "New projected distance: " << getDistance() << ", new azimuth: " << azimuth << std::endl;

	setNewSetpoints(ct_abspos, bearing - panOffset_ - magneticDeclination_, azimuth);
}

double GpsTrackingMode::getDistance() {
	return sqrt(targetEstimatedPosLocal_[0]*targetEstimatedPosLocal_[0]+targetEstimatedPosLocal_[1]*targetEstimatedPosLocal_[1]);
}

double GpsTrackingMode::getAzimuthFromLocal() {
	double groundDistance = getDistance();

	return rad2deg(atan2(targetEstimatedPosLocal_[2],groundDistance));
}

double GpsTrackingMode::getLOSdistance(GPSPos* posA, GPSPos* posB) {
	//		timespec startTs, endTs;
	//		clock_gettime(CLOCK_REALTIME,&startTs);
	double LOSdistance;
	double earthradius = 6378 * 1e3;

	double latA = deg2rad(posA->lat);
	double lonA = deg2rad(posA->lon);
	double rA = earthradius + posA->alt;
	double latB = deg2rad(posB->lat);
	double lonB = deg2rad(posB->lon);
	double rB = earthradius + posB->alt;

	double xA = rA * cos(latA) * cos(lonA);
	double yA = -rA * cos(latA) * sin(lonA);
	double zA = rA * sin(latA);

	double xB = rB * cos(latB) * cos(lonB);
	double yB = -rB * cos(latB) * sin(lonB);
	double zB = rB * sin(latB);

	LOSdistance = sqrt(
			(xA - xB) * (xA - xB) + (yA - yB) * (yA - yB)
					+ (zA - zB) * (zA - zB));
	//		clock_gettime(CLOCK_REALTIME,&endTs);
	//		std::cout << "update took " << ((endTs.tv_sec)-(startTs.tv_sec))*1e3 + ((endTs.tv_nsec)-(startTs.tv_nsec))*1e-6 << " ms" << std::endl;

	return LOSdistance;
}

double GpsTrackingMode::getDistance(GPSPos* posA, GPSPos* posB) {
	// assuming small distances and therefore
	// using a equirectangular approximation

	double distance, x, y;
	double earthradius = 6378 * 1e3;

	double latA = deg2rad(posA->lat);
	double lonA = deg2rad(posA->lon);
	double latB = deg2rad(posB->lat);
	double lonB = deg2rad(posB->lon);

	x = (lonB - lonA) * cos((latA + latB) / 2);
	y = (latB - latA);

	distance = sqrt(x * x + y * y) * earthradius;

	return distance;
}

double GpsTrackingMode::getAzimuth(GPSPos* posA, GPSPos* posB) {
	/*
	 double LOSdistance = getLOSdistance(posA,posB);
	 double dElev = posB->elev - posA->elev;
	 double quotient = 0;
	 if(LOSdistance(posA,posB) > 0)
	 quotient = dElev/LOSdistance;

	 if (quotient > 1)
	 quotient = 1;
	 else if (quotient < -1)
	 quotient = -1;

	 return rad2deg(asin(quotient));
	 */

	//assuming small distances
	double distance = getDistance(posA, posB);
	double dElev = posB->alt - posA->alt;

	return rad2deg(atan2(dElev, distance));

}

double GpsTrackingMode::getBearing(GPSPos* posA, GPSPos* posB) {
	// from http://www.movable-type.co.uk/scripts/latlong.html

	double bearing = 0;

	double d_lon = deg2rad(posB->lon - posA->lon);
	double lat_A = deg2rad(posA->lat);
	double lat_B = deg2rad(posB->lat);

	double y = sin(d_lon) * cos(lat_B);
	double x = cos(lat_A) * sin(lat_B) - sin(lat_A) * cos(lat_B) * cos(d_lon);
	bearing = rad2deg(atan2(y, x));

	bearing = fmod(bearing + 360.0, 360.0);

	return bearing;
}

double GpsTrackingMode::deg2rad(double deg) {
	return M_PI / 180.0 * deg;
}

double GpsTrackingMode::rad2deg(double rad) {
	return 180.0 / M_PI * rad;
}

std::ostream& operator<<(std::ostream& out,
		const GpsTrackingMode& gpsTracking) {
	out.precision(6);
	out << gpsTracking.targetPosLocal_[0] << " "
		<< gpsTracking.targetPosLocal_[1] << " "
		<< gpsTracking.targetPosLocal_[2] << " "
		<< gpsTracking.targetEstimatedPosLocal_[0] << " "
		<< gpsTracking.targetEstimatedPosLocal_[1] << " "
		<< gpsTracking.targetEstimatedPosLocal_[2] << " "
		<< gpsTracking.targetEstimatedPos_;
	return out;
}

} /* namespace tracking */
