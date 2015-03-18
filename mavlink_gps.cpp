/*
 * TSMavlinkGPS.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: asl
 */

#include <trackingsetup/mavlink_gps.h>

namespace tracking {

MavlinkGps::MavlinkGps(MavlinkMessages* mavlinkMessages, std::string label) :
        mavlinkMessages_(mavlinkMessages), lastTimestamp_(0) {
	this->setLabel(label);
}

bool MavlinkGps::getPos(GPSPos* pos) {
    if (mavlinkMessages_->gps_raw_int.fix_type < 3) {
//		pos->elev = -1;
//		pos->lat = 99;
//		pos->lon = 199;
		return false;
	}

    if (mavlinkMessages_->lastRawGpsPosition > lastTimestamp_) {
        pos->elev = mavlinkMessages_->gps_raw_int.alt / 1000.0;
        pos->lon = 1E-7 * mavlinkMessages_->gps_raw_int.lon;
        pos->lat = 1E-7 * mavlinkMessages_->gps_raw_int.lat;
        lastTimestamp_ = mavlinkMessages_->lastRawGpsPosition;
		return true;
	}
	return false;
}

float MavlinkGps::getPosAccuracy() {
	double posAccuracy = 100;

    if (mavlinkMessages_->gps_raw_int.fix_type == 3) {
		posAccuracy = sqrt(
                mavlinkMessages_->gps_raw_int.eph
                        * mavlinkMessages_->gps_raw_int.eph
                        + mavlinkMessages_->gps_raw_int.epv
                                * mavlinkMessages_->gps_raw_int.epv) / 100.0;
	}

	return posAccuracy;
}

int MavlinkGps::getFixType() {
    return mavlinkMessages_->gps_raw_int.fix_type;
}

std::ostream& operator<<(std::ostream& out, const MavlinkGps& mavlinkGPS) {
    MavlinkMessages mavlinkMessages = *mavlinkGPS.mavlinkMessages_;

	double posAccuracy = sqrt(
			mavlinkMessages.gps_raw_int.eph * mavlinkMessages.gps_raw_int.eph
					+ mavlinkMessages.gps_raw_int.epv
							* mavlinkMessages.gps_raw_int.epv) / 100;

	GPSPos pos;
	pos.elev = mavlinkMessages.gps_raw_int.alt / 1000.0;
	pos.lon = 1E-7 * mavlinkMessages.gps_raw_int.lon;
	pos.lat = 1E-7 * mavlinkMessages.gps_raw_int.lat;

	int numberOfSatellitesInUse = 0;
	for (int i = 0; i < 20; i++) {
		if (mavlinkMessages.gps_status.satellite_used[i] != 0) {
			numberOfSatellitesInUse++;
		}
	}

	std::string label;
	label = mavlinkGPS.label_;
	if (mavlinkMessages.lastRawGpsPosition > 0) {
        out << "$" << label << "Status ";
        out.precision(15);
        out << 0 << " " << ((int) mavlinkMessages.gps_raw_int.fix_type) << " ";
        out << posAccuracy << " ";
	}
    if (mavlinkMessages.lastGpsStatus > 0) {
        out << numberOfSatellitesInUse << " "
                << ((int) mavlinkMessages.gps_raw_int.satellites_visible);
    } else {
        out << 0 << " " << 0;
    }
	return out;
}

MavlinkGps::~MavlinkGps() {
	// TODO Auto-generated destructor stub
}

} /* namespace tracking */
