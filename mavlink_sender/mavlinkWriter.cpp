/*
 * TSmavlinkReader.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: asl
 */
#include "mavlinkWriter.h"

#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>
#include <iostream>
#include <string>
#include <sstream>


using namespace std;

namespace mavlinkSender {

mavlinkWriter::mavlinkWriter() :
		keepWritingMavlink_(true),
		vid_(0),
		pid_(0),
		interface_(0),
		label_(""),
		baudrate_(0),
		isOpen_(false),
		initialized_(false),
		writingStatus_(false),
		bytesWritten_(0),
		systemId_(0),
		autopilotId_(0),
		componentId_(0),
		writeThreadId_(0) {

	// init mutex
	pthread_mutex_init(&writeLock_, NULL);

}

mavlinkWriter::mavlinkWriter(int vid, int pid, int interface, int baudrate, const std::string label) :
		keepWritingMavlink_(true),
		vid_(vid),
		pid_(pid),
		interface_(interface),
		label_(label),
		baudrate_(baudrate),
		isOpen_(false),
		writingStatus_(false),
		bytesWritten_(0),
		systemId_(0),
		autopilotId_(0),
		componentId_(0),
		writeThreadId_(0) {

	pthread_mutex_init(&writeLock_, NULL);

	initialized_ = true;
}

mavlinkWriter::~mavlinkWriter() {
	pthread_mutex_destroy(&writeLock_);
}

void mavlinkWriter::init(int vid, int pid, int interface, int baudrate, const std::string label) {
	if(!initialized_) {
		vid_ = vid;
		pid_ = pid;
		interface_ = interface;
		baudrate_ = baudrate;
		label_ = label;
		initialized_ = true;
	}
	else {
		std::cout << "WARNING: Interface " << label_ << " already initialized!" << std::endl;
	}
}

bool mavlinkWriter::sendMessage(mavlink_message_t& message) {
	// method from github.com/mavlink/c_uart_interface_example -> serial_port.cpp

	unsigned char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	/* WRITE TO PORT */
	// Lock
	pthread_mutex_lock(&writeLock_);

	// Write packet via serial link
	int bytesWritten = context_.write(buf, len);

	// Unlock
	pthread_mutex_unlock(&writeLock_);

	bytesWritten_ += bytesWritten;

	return (bytesWritten > 0);
	// Done!
}



bool mavlinkWriter::startHeartbeatThread() {
	if (initialized_ != true) {
		std::cout << "mavlinkReader not properly initialized" << std::endl;
		return false;
	}
    std::cout << "starting thread writing MAVLINK heartbeat messages";
	if (writingStatus_ != false) {
		// printf(stderr,"read thread already running\n");
		// TODO: add some logging here
		return false;
	}

    if(pthread_create(&writeThreadId_, NULL, heartbeatThread, (void*) this)) {
    std::cout << "Started thread writing MAVLINK heartbeat messages" << std::endl;
		return true;
    }

	return false;

}

bool mavlinkWriter::openSerial() {

	std::stringstream logmessage("MavlinkReader " + label_ + " :: ");
	int res = 0;
	res = context_.set_interface((ftdi_interface) interface_);

	if( res < 0) {
		std::cout << "Unable to set interface number: " << context_.error_string() << std::endl;
		return false;
	}

	res = context_.open(vid_,pid_);
	if( res < 0) {
		std::cout << "Unable to open interface (" << res << "): " << context_.error_string() << std::endl;
		return false;
	}

	res =context_.set_baud_rate(baudrate_);
	if( res < 0) {
		std::cout << "Unable to set baud rate: " << context_.error_string() << std::endl;
		return false;
	}

	res = context_.set_line_property(BITS_8, STOP_BIT_1, NONE, BREAK_OFF);
	if( res < 0) {
		std::cout << "Unable to set line parameters: " << context_.error_string() << std::endl;
		return false;
	}

	res = context_.set_flow_control(SIO_DISABLE_FLOW_CTRL);
	if( res < 0) {
		std::cout << "Unable to set flow control: " << context_.error_string() << std::endl;
		return false;
	}

	isOpen_ = true;

	std::cout  << "Serial device "
			<< interface_ << " on (0x"
			<< std::hex << vid_
			<< ", 0x" << pid_ << std::dec
			<< ") successfully opened.";


	// Done!
	return true;

}

bool mavlinkWriter::closeSerial() {

	// TODO: add some logging
	int result = context_.close();

	if (result < 0) {
		// add some more logging here
		return false;
	}

	isOpen_ = false;
	return true;
}

void mavlinkWriter::stopSending() {
	keepWritingMavlink_ = false;

//	pthread_join(writeThreadId_, NULL);
}

//void* mavlinkSender::writeThread(void* arg) {
//	mavlinkSender *mavlinkSender = (mavlinkSender *) arg;
//
//	/* variable to check whether MAVLINK reader is kept alive */
//	bool doKeepWriting = mavlinkSender->keepWriting();
//
//	mavlinkSender->writingStatus_ = true;
//
//	while (doKeepWriting) {
//		mavlinkSender->readMessages();
//		usleep(10000); // Read batches at 100Hz
//
//		/* check if Server should be closed */
//		doKeepWriting = mavlinkSender->keepWriting();
//	}
////	mavlinkReader->addLogMessage(vl_DEBUG, "MAVLINK reading thread stopped.");
//	mavlinkSender->writingStatus_ = false;
//
//	return NULL;
//
//}

void* mavlinkWriter::heartbeatThread(void* arg) {
    mavlinkWriter* writer = (mavlinkWriter*) arg;

    /* variable to check whether MAVLINK reader is kept alive */
    bool doKeepWriting = writer->keepWriting();

    writer->writingStatus_ = true;

    mavlink_message_t msg;

    mavlink_heartbeat_t heartbeatMsg;
    heartbeatMsg.type = 1;
    heartbeatMsg.autopilot = 15;
    heartbeatMsg.base_mode = 193;
    heartbeatMsg.custom_mode = 65536;
    heartbeatMsg.system_status = 3;
    heartbeatMsg.mavlink_version = 3;

    while (doKeepWriting) {
        mavlink_msg_heartbeat_encode(1,255,&msg,&heartbeatMsg);

        writer->sendMessage(msg);
        usleep(450000);

        /* check if Server should be closed */
        doKeepWriting = writer->keepWriting();
    }
    writer->writingStatus_ = false;

    return NULL;

}

int mavlinkWriter::getBytesWritten() {
    return bytesWritten_;
}

bool mavlinkWriter::keepWriting() {
	return keepWritingMavlink_;
}

} /* namespace tracking */
