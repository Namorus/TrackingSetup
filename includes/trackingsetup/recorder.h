/*
 * TARecorder.h
 *
 *  Created on: Jun 25, 2013
 *      Author: thomas
 */

#ifndef TARECORDER_H_
#define TARECORDER_H_

#include <trackingsetup/trackingsetup.h>

namespace tracking {

class Recorder: public tracking::TrackingSetup {
public:
	Recorder();

	int start(recorderSettings& settings);
	void stop();

	bool isRecording();

	void record(timespec* const Ts, std::vector<float>* const RSSvalues,
			int panPosition, int tiltPosition, setpoints* motorSetPoints, GPSPos* const localGPS,
			GPSPos* const remoteGPS, GpsTrackingMode* const gpsTracking, RadioRSSI* const radioRssi, int curMode);

private:

	std::ofstream fileHandle;
	bool recording;

	recorderSettings settings;

	pthread_mutex_t* pRecordMutex;

	void writeHeader();
};

} /* namespace tracking */
#endif /* TARECORDER_H_ */
