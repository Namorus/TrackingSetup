/*
 * TSmavlinkReader.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: asl
 */
#include <sstream>
#include <string>
#include <sys/time.h>
#include <unistd.h>

#include "trackingsetup/mavlink_reader.h"


namespace tracking {

MavlinkReader::MavlinkReader() :
				keepReadingMavlink_(true),
				vid_(0),
				pid_(0),
				interface_(0),
				label_(""),
				baudrate_(0),
				isOpen_(false),
				readingStatus_(false),
				systemId_(0),
				autopilotId_(0),
				componentId_(0),
				readThreadId_(0) {

	// Start mutex
	pthread_mutex_init(&readLock_, NULL);
	pthread_mutex_init(&messagesLock_, NULL);

	initialized_ = false;
}

MavlinkReader::MavlinkReader(int vid, int pid, int interface, int baudrate, const std::string label) :
		keepReadingMavlink_(true),
		vid_(vid),
		pid_(pid),
		interface_(interface),
		baudrate_(baudrate),
		isOpen_(false),
		readingStatus_(false),
		systemId_(0),
		autopilotId_(0),
		componentId_(0),
		readThreadId_(0) {

	pthread_mutex_init(&readLock_, NULL);
	pthread_mutex_init(&messagesLock_, NULL);

	label_ = label;
	initialized_ = true;
}

MavlinkReader::~MavlinkReader() {
	pthread_mutex_destroy(&readLock_);
	pthread_mutex_destroy(&messagesLock_);
}

void MavlinkReader::init(int vid, int pid, int interface, int baudrate, const std::string label) {
	vid_ = vid;
	pid_ = pid;
	interface_ = interface;
	baudrate_ = baudrate;
	label_ = label;
	initialized_ = true;
}

bool MavlinkReader::start() {
	if (initialized_ != true) {
		addLogMessage(vl_ERROR, "mavlinkReader not properly initialized");
		return false;
	}

	if (readingStatus_ != false) {
		addLogMessage(vl_ERROR, "mavlinkReader already started");
		return false;
	}

	if (!openSerial()) {
		addLogMessage(vl_ERROR, "MavlinkReader :: Could not open serial port " + label_);
	} else {
		// open serial port
		pthread_create(&readThreadId_, NULL, readThread, (void*) this);
		addLogMessage(vl_INFO, "MavlinkReader :: Started thread reading MAVLINK messages on " + label_);
		return true;
	}

	return false;
}

bool MavlinkReader::openSerial() {
	std::stringstream logmessage("MavlinkReader " + label_ + " :: ");
	int res = 0;
	res = context_.set_interface((ftdi_interface) interface_);

	if( res < 0) {
		logmessage << "Unable to set interface number: " << context_.error_string();
		addLogMessage(vl_ERROR,logmessage.str());
		return false;
	}

	res = context_.open(vid_,pid_);
	if( res < 0) {
		logmessage << "Unable to open interface (" << res << "): " << context_.error_string();
		addLogMessage(vl_ERROR,logmessage.str());
		return false;
	}

	res =context_.set_baud_rate(baudrate_);
	if( res < 0) {
		logmessage << "Unable to set baud rate: " << context_.error_string();
		addLogMessage(vl_ERROR,logmessage.str());
		return false;
	}

	res = context_.set_line_property(BITS_8, STOP_BIT_1, NONE, BREAK_OFF);
	if( res < 0) {
		logmessage << "Unable to set line parameters: " << context_.error_string();
		addLogMessage(vl_ERROR,logmessage.str());
		return false;
	}

	res = context_.set_flow_control(SIO_DISABLE_FLOW_CTRL);
	if( res < 0) {
		logmessage << "Unable to set flow control: " << context_.error_string();
		addLogMessage(vl_ERROR,logmessage.str());
		return false;
	}

	lastStatus_.packet_rx_drop_count = 0;

	isOpen_ = true;

	logmessage << "Serial device "
			<< interface_ << " on (0x"
			<< std::hex << vid_
			<< ", 0x" << pid_ << std::dec
			<< ") successfully opened.";
	addLogMessage(vl_INFO,logmessage.str());

	// Done!
	return true;

}

bool MavlinkReader::closeSerial() {

	// TODO: add some logging
	int result = context_.close();

	if (result < 0) {
		// add some more logging here
		return false;
	}

	isOpen_ = false;
	return true;
}

void* MavlinkReader::readThread(void* arg) {
	MavlinkReader *mavlinkReader = (MavlinkReader *) arg;

	/* variable to check whether MAVLINK reader is kept alive */
	bool doKeepReading = mavlinkReader->keepReading();

    mavlinkReader->readingStatus_ = true;

    while (doKeepReading) {
        mavlinkReader->readMessages();
        usleep(10000); // Read batches at 100Hz

		/* check if Server should be closed */
		doKeepReading = mavlinkReader->keepReading();
	}
	mavlinkReader->addLogMessage(vl_DEBUG, "MAVLINK reading thread stopped.");
	mavlinkReader->readingStatus_ = false;

	return NULL;
}

void MavlinkReader::readMessages() {
	uint8_t cp;
	mavlink_status_t status;
	mavlink_message_t message;
	int result = 1;
	// Blocking wait for new data
	while (result > 0) {
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------

		uint8_t msgReceived = false;

		/* READ FROM PORT */
		// Lock
		pthread_mutex_lock(&readLock_);

		// read
		result = context_.read(&cp,1);

		// Unlock
		pthread_mutex_unlock(&readLock_);

		/* PARSE MESSAGE */
		if (result > 0) {
			// the parsing
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

			lastStatus_ = status;
		} else if(result == 0) { // no data

		} else { 	// Couldn't read from port
			addLogMessage(vl_ERROR, "Could not read from serial device " + label_);
		}

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if (msgReceived) {
			messageHandler(message);
		}

	}

	return;
}

void MavlinkReader::messageHandler(mavlink_message_t& message) {
	std::stringstream logmessage("MavlinkReader " + label_ + " :: ");

	pthread_mutex_lock(&messagesLock_);

	// Store message sysid and compid.
	// Note this doesn't handle multiple message sources.
	currentMessages_.sysid = message.sysid;
	currentMessages_.compid = message.compid;

	// Handle Message ID
	switch (message.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT: {
			mavlink_msg_heartbeat_decode(&message,&(currentMessages_.heartbeat));
			currentMessages_.lastHeartbeat = get_usec();
			break;
		}

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
			mavlink_msg_global_position_int_decode(&message,&(currentMessages_.global_position_int));
			currentMessages_.lastGlobalPosition = get_usec();
			logmessage << "received GPOS message @" << get_usec();
			addLogMessage(vl_DEBUG,logmessage.str());
			break;
		}

		case MAVLINK_MSG_ID_GPS_RAW_INT: {
			mavlink_msg_gps_raw_int_decode(&message,&(currentMessages_.gps_raw_int));
			currentMessages_.lastRawGpsPosition = get_usec();
			break;
		}

		case MAVLINK_MSG_ID_GPS_STATUS: {
			mavlink_msg_gps_status_decode(&message,&(currentMessages_.gps_status));
			currentMessages_.lastGpsStatus = get_usec();
			break;
		}

		case MAVLINK_MSG_ID_HIGHRES_IMU: {
			mavlink_msg_highres_imu_decode(&message,&(currentMessages_.highres_imu));
			currentMessages_.lastHighresImu = get_usec();
			break;
		}

		case MAVLINK_MSG_ID_ATTITUDE: {
			//printf("MAVLINK_MSG_ID_ATTITUDE\n");
			mavlink_msg_attitude_decode(&message,&(currentMessages_.attitude));
			currentMessages_.lastAttitude = get_usec();
			break;
		}

		case MAVLINK_MSG_ID_RADIO_STATUS: {
			mavlink_msg_radio_status_decode(&message,&(currentMessages_.radio_status));
			currentMessages_.lastRadioStatus = get_usec();
			break;
		}

		default: {
			// printf("Warning, did not handle message id %i\n",message.msgid);
			break;
		}

	} // end: switch msgid
	pthread_mutex_unlock(&messagesLock_);
}

bool MavlinkReader::keepReading() {
	return keepReadingMavlink_;
}

void MavlinkReader::stopReading() {
	keepReadingMavlink_ = false;
	usleep(100000);
	pthread_join(readThreadId_, NULL);
}

void MavlinkReader::getMavlinkMessages(MavlinkMessages& messages) {
	pthread_mutex_lock(&messagesLock_);
	messages = currentMessages_;
	pthread_mutex_unlock(&messagesLock_);
}

int MavlinkReader::getRxDropCount() {
	return lastStatus_.packet_rx_drop_count;
}

int MavlinkReader::getRxSuccessCount() {
	return lastStatus_.packet_rx_success_count;
}

uint64_t MavlinkReader::get_usec() {
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

} /* namespace tracking */
