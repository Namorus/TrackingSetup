/*
 * TAGPSTracking.cpp
 *
 *  Created on: Jun 18, 2013
 *      Author: thomas
 */

#include <trackingsetup/gps_tracking.h>

namespace tracking {

GpsTrackingMode::GpsTrackingMode() :
		panOffset_(0), tiltOffset_(0), magneticDeclination_(0) {

}

void GpsTrackingMode::setMapping(float panOffset, float tiltOffset) {
	panOffset_ = panOffset;
	tiltOffset_ = tiltOffset;
}

void GpsTrackingMode::setMagneticDeclination(float magneticDeclination) {
	magneticDeclination_ = magneticDeclination;
}

void GpsTrackingMode::update(GPSPos& antennaPos, GPSPos& targetPos) {
	double bearing = getBearing(&antennaPos, &targetPos);
	double azimuth = getAzimuth(&antennaPos, &targetPos);

	/*
	 * tiltOffset was wrongly calculated, and is omitted for the time being.
	 *
	 * setNewSetpoints(ct_abspos,bearing-panOffset,azimuth-tiltOffset);
	 */
	setNewSetpoints(ct_abspos, bearing - panOffset_ - magneticDeclination_, azimuth);
}


double GpsTrackingMode::getLOSdistance(GPSPos* posA, GPSPos* posB) {
	//		timespec startTs, endTs;
	//		clock_gettime(CLOCK_REALTIME,&startTs);
	double LOSdistance;
	double earthradius = 6378 * 1e3;

	double latA = deg2rad(posA->lat);
	double lonA = deg2rad(posA->lon);
	double rA = earthradius + posA->elev;
	double latB = deg2rad(posB->lat);
	double lonB = deg2rad(posB->lon);
	double rB = earthradius + posB->elev;

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
	double dElev = posB->elev - posA->elev;

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


} /* namespace tracking */
