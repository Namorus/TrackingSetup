/*
 * TARecorder.h
 *
 *  Created on: Jun 25, 2013
 *      Author: thomas
 */

#ifndef TARECORDER_H_
#define TARECORDER_H_

#include "TAClass.h"

namespace tracking {

class TARecorder: public tracking::TAClass {
public:
	TARecorder();

	int start(recorderSettings& settings);
	void stop();

	bool isRecording();

	void record(timespec* const Ts, std::vector<float>* const RSSvalues,
			int panPosition, int tiltPosition, GPSPos* const localGPS,
			GPSPos* const remoteGPS, int curMode);

private:

	std::ofstream fileHandle;
	bool recording;

	recorderSettings settings;

	pthread_mutex_t* pRecordMutex;

	void writeHeader();
};

} /* namespace tracking */
#endif /* TARECORDER_H_ */
