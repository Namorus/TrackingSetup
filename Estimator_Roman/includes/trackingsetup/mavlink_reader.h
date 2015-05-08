/*
 * mavlinkReader.h
 *
 *  Created on: Jan 29, 2015
 *      Author: Thomas Mantel
 *  Last Change on: Mar 13, 2015
 */

#ifndef MAVLINKREADER_H_
#define MAVLINKREADER_H_

#include <common/mavlink.h>
#include <ftdi.hpp>

#include "trackingsetup/input.h"
#include "trackingsetup/types.h"

namespace tracking {

class MavlinkReader: public Input {
public:
	MavlinkReader();
	MavlinkReader(int vid, int pid, int interface, int baudrate, const std::string label);
	virtual ~MavlinkReader();

	void init(int vid, int pid, int interface, int baudrate, const std::string label);
	bool start();

	bool openSerial();
	bool closeSerial();

	static void* readThread(void* arg);

	void readMessages();
	void messageHandler(mavlink_message_t& message);

	bool keepReading();
	void stopReading();

	void getMavlinkMessages(MavlinkMessages &messages);
    int getRxDropCount();
    int getRxSuccessCount();

    static uint64_t get_usec();

private:
	bool keepReadingMavlink_;

	int vid_;
	int pid_;
	int interface_;
	std::string label_;

	int baudrate_;
	bool isOpen_;

	Ftdi::Context context_;

	bool initialized_;
	bool readingStatus_;

	int systemId_;
	int autopilotId_;
	int componentId_;

	MavlinkMessages currentMessages_;

	// Thread ID of reading thread
	pthread_t readThreadId_;

	// mutex to lock data of read thread
	pthread_mutex_t readLock_;

	// mutex to lock struct MavlinkMessages
	pthread_mutex_t messagesLock_;

	mavlink_status_t lastStatus_;

};

} /* namespace tracking */

#endif /* MAVLINKREADER_H_ */
