/*
 * TSmavlinkReader.h
 *
 *  Created on: Jan 29, 2015
 *      Author: asl
 */

#ifndef MAVLINKWRITER_H_
#define MAVLINKWRITER_H_

#include <pthread.h>
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <string>
#include <libftdi1/ftdi.hpp>
#include <common/mavlink.h>

namespace mavlinkSender {

class mavlinkWriter {
public:
	mavlinkWriter();
	mavlinkWriter(int vid, int pid, int interface, int baudrate, const std::string label);
	virtual ~mavlinkWriter();

	void init(int vid, int pid, int interface, int baudrate, const std::string label);
    bool startHeartbeatThread();

	bool sendMessage(mavlink_message_t &message);

	bool openSerial();
	bool closeSerial();

	void stopSending();

    static void* heartbeatThread(void* arg);

	bool keepWriting();

    int getBytesWritten();

private:
	bool keepWritingMavlink_;

	int vid_;
	int pid_;
	int interface_;
	std::string label_;

	int baudrate_;
	bool isOpen_;

	Ftdi::Context context_;

	bool initialized_;
	bool writingStatus_;

	int bytesWritten_;

	int systemId_;
	int autopilotId_;
	int componentId_;

	// Thread ID of reading thread
	pthread_t writeThreadId_;

	// mutex to lock data of read thread
	pthread_mutex_t writeLock_;
};

} /* namespace mavlinkSender */

#endif /* MAVLINKWRITER_H_ */
