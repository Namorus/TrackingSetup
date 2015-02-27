/*
 * TSmavlinkReader.h
 *
 *  Created on: Jan 29, 2015
 *      Author: asl
 */

#ifndef TSMAVLINKREADER_H_
#define TSMAVLINKREADER_H_

#include "TAinput.h"
#include "TAtypes.h"
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <common/mavlink.h>

namespace tracking {

class TSmavlinkReader: public TAinput {
public:
	TSmavlinkReader();
	TSmavlinkReader(std::string &port, int &baudrate);
	virtual ~TSmavlinkReader();

	void init(std::string &port, int &baudrate);
	bool start();

	int readMessage(mavlink_message_t &message);
	void readMessages();

	bool openSerial();
	bool closeSerial();

	void stopReading();

	void getMavlinkMessages(MavlinkMessages &messages);

	static void* readThread(void* arg);

	bool keepReading();

    static uint64_t get_usec();

private:
	bool keepReadingMavlink_;

	std::string port_;
	int baudrate_;
	bool isOpen_;

	int fd_;

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

	bool setupSerial();

};

} /* namespace tracking */

#endif /* TSMAVLINKREADER_H_ */
