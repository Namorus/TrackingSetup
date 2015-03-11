/*
 * TrackingAntenna.h
 *
 *  Created on: Jun 15, 2013
 *      Author: thomas
 */


#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <common/mavlink.h>


#ifndef TRACKINGANTENNA_H_
#define TRACKINGANTENNA_H_

namespace tracking {

enum verbosityLevel {
	vl_ERROR = 0, vl_WARNING = 1, vl_INFO = 2, vl_DEBUG = 3
};

struct TALogMessage {
	time_t timestamp;
	verbosityLevel vLvl;
	std::string message;
};

enum WLANFilter {
	flt_usemostrecent = 0, flt_mean = 1, flt_median = 2
};

struct commandLineOptions {
	commandLineOptions() :
			cfgFileName("trackingsetup.cfg"), logFileName(
					"trackingsetup.log"), verbosity(vl_ERROR), noMotors(
					false), noLocalGPS(false), noRemoteGPS(false), noWLAN(
					false), useTUI(false), useStdout(false) {
	}

	std::string cfgFileName;
	std::string logFileName;
	verbosityLevel verbosity;
	bool noMotors;
	bool noLocalGPS;
	bool noRemoteGPS;
	bool noWLAN;
	bool useTUI;
	bool useStdout;
};

struct GPSPos {
	double lat;
	double lon;
	double elev;

	GPSPos() :
			lat(99.0), lon(199.0), elev(-1) {
	}

	std::string toString() {
		char temp[50];
		memset(temp, '\0', 50);
		sprintf(temp, "(%.4fN/%.4fE @ %.2fm)", lat, lon, elev);

		std::string posString = temp;

		return posString;
	}

	friend std::ostream& operator<<(std::ostream& out, const GPSPos& pos) {
		out.precision(15);
		out << pos.lat << " " << pos.lon << " " << pos.elev;
		return out;
	}
	friend std::istream& operator>>(std::istream& in, GPSPos& pos) {
		in >> pos.lat;
		in >> pos.lon;
		in >> pos.elev;
		return in;
	}
	friend bool operator==(const GPSPos& a, const GPSPos& b) {
		if (a.lat == b.lat && a.lon == b.lon && a.elev == b.elev)
			return true;
		return false;
	}
	friend bool operator!=(const GPSPos& a, const GPSPos& b) {
		if (a == b)
			return false;
		return true;
	}

};

struct GlobalConf {
	friend bool operator==(const GlobalConf& a, const GlobalConf& b);
	friend bool operator!=(const GlobalConf& a, const GlobalConf& b);
	int verbose;
//	bool use_ncurses;
	int updateFreq;
//	int GuiUpdateFreq;
};

struct GPStrackingConf {
	friend bool operator==(const GPStrackingConf& a, const GPStrackingConf& b);
	friend bool operator!=(const GPStrackingConf& a, const GPStrackingConf& b);
	GPSPos AntennaPos;

	std::string localMavlinkPort;
	int localMavlinkBaudrate;

	std::string remoteMavlinkPort;
	int remoteMavlinkBaudrate;
};

struct MotorControlConf {
	friend bool operator==(const MotorControlConf& a,
			const MotorControlConf& b);
	friend bool operator!=(const MotorControlConf& a,
			const MotorControlConf& b);
	std::string panEposNo;
	std::string tiltEposNo;
	int commRetries;
};

struct LocateConf {
	LocateConf() :
			panSpeed(0), tiltAngleStart(0), tiltAngleStop(0), pitch(0) {
	}
	friend bool operator==(const LocateConf& a, const LocateConf& b);
	friend bool operator!=(const LocateConf& a, const LocateConf& b);
	float panSpeed;
	float tiltAngleStart;
	float tiltAngleStop;
	float pitch;
};

struct FindNorthConf {
	FindNorthConf() :
		panSpeed(0), tiltAngle(0) {}
	friend bool operator==(const FindNorthConf& a, const FindNorthConf&b);
	friend bool operator!=(const FindNorthConf& a, const FindNorthConf&b);
	float panSpeed;
	float tiltAngle;
};

enum trackingSetupState {
	ts_ENDING = -1,
	ts_INIT = 0,
	ts_GPS_TRACKING = 1,
	ts_LOCATE = 2,
	ts_FIND_NORTH = 8,
	ts_READY = 9,
	ts_STOP = 10
};

enum motorControlType {
	ct_undefined = -1,
	ct_velocity = 1,
	ct_abspos = 2,
	ct_relpos = 3,
	ct_init = 10
};

enum findNorthState {
	fn_NOTREADY = -1,
	fn_INITIALIZING = 0,
	fn_RECORDING = 1,
	fn_EVALUATING = 2,
	fn_FINISHED = 9
};

enum locateState {
	lcte_NOTREADY = -1,
	lcte_INITIALIZING = 0,
	lcte_READY = 1,
	lcte_LOCATING = 2,
	lcte_EVALUATING = 3,
	lcte_POSITIONING = 4,
	lcte_FINISHED = 9
};

//struct setpoint {
//	setpoint() :
//		type (ct_undefined),
//		pan (0.0),
//		tilt (0.0) {
//		// initializer
//	}
//	friend std::ostream& operator<<(std::ostream& out, const setpoint& a) {
//		out << "$Setpoint ";
//		out.precision(4);
//		out << (int) a.type << " " << a.pan << " " << a.tilt;
//		return out;
//	}
//	TActrlType type;
//	double pan;
//	double tilt;
//};

struct setpoints {
	setpoints() :
			panCtrltype(ct_undefined), panValue(0.0), tiltCtrltype(
					ct_undefined), tiltValue(0.0) {
	} // initializer

	friend std::ostream& operator<<(std::ostream& out, const setpoints& a) {
		out << "$Setpoints ";
		out.precision(6);
		out << (int) a.panCtrltype << " " << a.panValue << " "
				<< (int) a.tiltCtrltype << " " << a.tiltValue;
		return out;
	}
	motorControlType panCtrltype;
	double panValue;
	motorControlType tiltCtrltype;
	double tiltValue;
};

struct configChanges {
	bool anyChanges;
	bool Glbl;
	bool RSSI;
	bool GPS;
	bool Mot;
	bool WLAN;
	bool RSStracking;
	bool locate;

	configChanges() :
			anyChanges(false), Glbl(false), RSSI(false), GPS(false), Mot(false), WLAN(
					false), RSStracking(false), locate(false) {
	}
	;
};

struct recorderSettings {
	bool recordTimestamp;
	bool recordRSSI;
	bool recordMotData;
	bool recordLocalGPS;
	bool recordRemoteGPS;
	bool recordCurMode;
	std::vector<std::string> wlanDevNames;

	recorderSettings() :
			recordTimestamp(false), recordRSSI(false), recordMotData(false), recordLocalGPS(
					false), recordRemoteGPS(false), recordCurMode(false) {
	} // initializer

	friend std::istream& operator>>(std::istream& in, recorderSettings& a) {

		in >> a.recordTimestamp;
		in >> a.recordRSSI;
		in >> a.recordMotData;
		in >> a.recordLocalGPS;
		in >> a.recordRemoteGPS;
		in >> a.recordCurMode;
		return in;
	}
};

// structure to store most recent message
// and other data about system
struct MavlinkMessages {
    MavlinkMessages() :
        lastHeartbeat(0),
        lastGlobalPosition(0),
        lastRawGpsPosition(0),
        lastGpsStatus(0),
        lastHighresImu(0),
        lastAttitude(0),
        lastRadioStatus(0) {}
	int sysid;
	int compid;
	// Heartbeat
	mavlink_heartbeat_t heartbeat;
    uint64_t lastHeartbeat;

	// Global Position
	mavlink_global_position_int_t global_position_int;
    uint64_t lastGlobalPosition;

	// RAW GPS Position
	mavlink_gps_raw_int_t gps_raw_int;
    uint64_t lastRawGpsPosition;

	// GPS Status
	mavlink_gps_status_t gps_status;
    uint64_t lastGpsStatus;

	// IMU (magnetometer) data
	mavlink_highres_imu_t highres_imu;
    uint64_t lastHighresImu;

	// Attitude
	mavlink_attitude_t attitude;
    uint64_t lastAttitude;

	// Radio info
	mavlink_radio_status_t radio_status;
    uint64_t lastRadioStatus;
};

struct MagReading {
	MagReading() :
		magX(0), magY(0), magZ(0) {}
	float magX;
	float magY;
	float magZ;

	friend std::ostream& operator<<(std::ostream& out, const MagReading& magReading) {
		out.precision(15);
		out << magReading.magX << " " << magReading.magY << " " << magReading.magZ;
		return out;
	}
	friend std::istream& operator>>(std::istream& in, MagReading& magReading) {
		in >> magReading.magX;
		in >> magReading.magX;
		in >> magReading.magX;
		return in;
	}
};


struct RadioRSSI {
    RadioRSSI() :
        rssi(0), remRssi(0), txbuf(0), noise(0), remNoise(0), rxErrors(0), fixed(0) {}
    int rssi;
    int remRssi;
    int txbuf;
    int noise;
    int remNoise;
    int rxErrors;
    int fixed;

//	friend std::ostream& operator<<(std::ostream& out, const MagReading& magReading) {
//		out.precision(15);
//		out << magReading.magX << " " << magReading.magY << " " << magReading.magZ;
//		return out;
//	}
//	friend std::istream& operator>>(std::istream& in, MagReading& magReading) {
//		in >> magReading.magX;
//		in >> magReading.magX;
//		in >> magReading.magX;
//		return in;
//	}
};

struct GlobalPos {
	GlobalPos() :
		timestamp(0), localTimestamp(0) {
		speed.lat = 0;
		speed.lon = 0;
		speed.elev = 0;
	}
	GPSPos position;
	GPSPos speed;

	uint32_t timestamp;
	uint64_t localTimestamp;
};

}
#endif /* TRACKINGANTENNA_H_ */
