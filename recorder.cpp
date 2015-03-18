/*
 * TARecorder.cpp
 *
 *  Created on: Jun 25, 2013
 *      Author: thomas
 */


#include <sstream>

#include <trackingsetup/gps_tracking.h>
#include <trackingsetup/recorder.h>


namespace tracking {

Recorder::Recorder() :
		recording(false) {
	pRecordMutex = new pthread_mutex_t;
	pthread_mutex_init(pRecordMutex, NULL);
}

int Recorder::start(recorderSettings &_settings) {
	if (!isRecording()) {
		settings = _settings;
		std::stringstream filename;
		filename << "datarecording_" << time(NULL) << ".csv";
		fileHandle.open(filename.str().c_str(), std::ofstream::out);
		if (fileHandle.is_open() && fileHandle.good()) {
			addLogMessage(vl_INFO,
					std::string("TARecorder: Successfully opened ")
							+ filename.str()
							+ std::string(". Start recording."));
			writeHeader();
			recording = true;
			return 0;
		}
		addLogMessage(vl_WARNING,
				std::string("TARecorder: Could not start recording. ")
						+ filename.str() + " could not be opened.");
		return -1;
	}
	addLogMessage(vl_WARNING,
			std::string(
					"TARecorder: Ignored request to start recording already recording."));
	return -2;
}

void Recorder::stop() {
	pthread_mutex_lock(pRecordMutex);
	fileHandle.close();
	recording = false;
	pthread_mutex_unlock(pRecordMutex);
	if (!fileHandle.is_open()) {
		addLogMessage(vl_INFO,
				std::string("TARecorder: Stopped recording and closed file."));
	}
}

bool Recorder::isRecording() {
	pthread_mutex_lock(pRecordMutex);
	bool temp = recording;
	pthread_mutex_unlock(pRecordMutex);

	return temp;
}

void Recorder::record(timespec* const Ts, std::vector<float>* const RSSvalues,
		int panPosition, int tiltPosition, setpoints* motorSetPoints, GPSPos* const localGPS,
		GPSPos* const remoteGPS, GpsTrackingMode* const gpsTracking, int curMode) {
	if (isRecording()) {
		// prepare string to write

		std::stringstream dataline;
		if (settings.recordTimestamp) {
			double timestamp = ((double) Ts->tv_sec)
					+ (((double) Ts->tv_nsec) / (double) 1e9);
			dataline.precision(13);
			dataline << timestamp << " ";
		}
		if (settings.recordRSSI) {
			dataline.precision(6);
			for (std::vector<float>::iterator rssIt = RSSvalues->begin();
					rssIt != RSSvalues->end(); rssIt++) {
				dataline << *rssIt << " ";
			}
		}

		if (settings.recordMotData) {
			dataline.precision(6);
			dataline << panPosition << " " << tiltPosition << " ";
			dataline << (int) motorSetPoints->panCtrltype << " " << motorSetPoints->panValue << " "
					 << (int) motorSetPoints->tiltCtrltype << " " << motorSetPoints->tiltValue << " ";

		}

		if (settings.recordLocalGPS) {
			dataline << *localGPS << " ";
		}

		if (settings.recordRemoteGPS) {
			dataline << *remoteGPS << " ";
		}

		if (settings.recordGPSTracking) {
			dataline << *gpsTracking << " ";
		}

		if (settings.recordCurMode) {
			dataline << curMode << " ";
		}

		// write data
		pthread_mutex_lock(pRecordMutex);
		if (fileHandle.good() && fileHandle.is_open()) {
			fileHandle << dataline.str() << std::endl;
		} else {
			addLogMessage(vl_WARNING,
					"TARecorder: Could not write data to file");
		}
		pthread_mutex_unlock(pRecordMutex);
	}

}

void Recorder::writeHeader() {
	std::stringstream headerLine;
	if (settings.recordTimestamp) {
		headerLine << "timestamp ";
	}
	if (settings.recordRSSI) {

	}

	if (settings.recordMotData) {
		headerLine << "panPosition tiltPosition ";
		headerLine << "panCtrlType panSetValue" << " "
				 << "tiltCtrlType tiltSetValue" << " ";
	}

	if (settings.recordLocalGPS) {
		headerLine << "local.lat local.lon local.elev ";
	}

	if (settings.recordRemoteGPS) {
		headerLine << "object.lat object.lon object.elev ";
	}


	if (settings.recordGPSTracking) {
		headerLine << "object.x object.y object.z "
				   << "estObject.x estObject.y estObject.z "
				   << "estObject.lat estObject.lon estObject.elev ";
	}
	if (settings.recordCurMode) {
		headerLine << "curMode";
	}


	pthread_mutex_lock(pRecordMutex);
//	std::cout << headerLine.str() << std::endl;
	fileHandle << headerLine.str() << std::endl;
	pthread_mutex_unlock(pRecordMutex);

}

} /* namespace tracking */
