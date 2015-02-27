/*
 * TSmavlinkReader.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: asl
 */
#include <fcntl.h>   // File control definitions
#include <string>
#include <sstream>
#include <sys/time.h>
#include <termios.h> // POSIX terminal control definitions
#include <trackingsetup/mavlink_reader.h>
#include <unistd.h>


namespace tracking {

TSmavlinkReader::TSmavlinkReader() :
		keepReadingMavlink_(true), port_(""), baudrate_(0), isOpen_(false), fd_(
				0), initialized_(false), readingStatus_(false), systemId_(0), autopilotId_(
				0), componentId_(0), readThreadId_(0) {

	// Start mutex
	pthread_mutex_init(&readLock_, NULL);
	pthread_mutex_init(&messagesLock_, NULL);

}

TSmavlinkReader::TSmavlinkReader(std::string& port, int& baudrate) :
		keepReadingMavlink_(true), port_(port), baudrate_(baudrate), isOpen_(
				false), fd_(0), readingStatus_(false), systemId_(0), autopilotId_(
				0), componentId_(0), readThreadId_(0) {

	pthread_mutex_init(&readLock_, NULL);
	pthread_mutex_init(&messagesLock_, NULL);

	initialized_ = true;
}

TSmavlinkReader::~TSmavlinkReader() {
	pthread_mutex_destroy(&readLock_);
	pthread_mutex_destroy(&messagesLock_);

}

void TSmavlinkReader::init(std::string& port, int& baudrate) {
	port_ = port;
	baudrate_ = baudrate;
	initialized_ = true;
}

int TSmavlinkReader::readMessage(mavlink_message_t& message) {
	// method from github.com/mavlink/c_uart_interface_example -> serial_port.cpp

	uint8_t cp;
	mavlink_status_t status;
	uint8_t msgReceived = false;

	/* READ FROM PORT */
	// Lock
	pthread_mutex_lock(&readLock_);

	// read
	int result = read(fd_, &cp, 1);

	// Unlock
	pthread_mutex_unlock(&readLock_);

	/* PARSE MESSAGE */
	if (result > 0) {
		// the parsing
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

		// check for dropped packets
		if (lastStatus_.packet_rx_drop_count != status.packet_rx_drop_count) {
			// printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
//            std::stringstream logmessage;
//            logmessage << port_ << ": Dropped " << ((int) status.packet_rx_drop_count) << " packets";
//            addLogMessage(vl_WARNING,logmessage.str());

// TODO: add some debug message here
//			unsigned char v=cp;
//			fprintf(stderr,"%02x ", v);
		}
		lastStatus_ = status;
	}

	// Couldn't read from port
	else {
		addLogMessage(vl_ERROR, "Cout not read from serial device");
//		fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
		// TODO: add some debug message here
	}

	// Done!
	return msgReceived;

}

void TSmavlinkReader::readMessages() {
	// method from github.com/mavlink/c_uart_interface_example -> autopilot_interface.cpp
	bool success = false;               // receive success flag
	mavlink_message_t message;

	// Blocking wait for new data
	while (not success and keepReadingMavlink_) {
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		success = readMessage(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if (success) {
			pthread_mutex_lock(&messagesLock_);
			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			currentMessages_.sysid = message.sysid;
			currentMessages_.compid = message.compid;
//            if(port_ == "/dev/ttyUSB0")
//                printf("#%d: ",message.msgid);
			// Handle Message ID
			switch (message.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT: {
//                printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                mavlink_msg_heartbeat_decode(&message,&(currentMessages_.heartbeat));
                currentMessages_.lastHeartbeat = get_usec();
//					current_messages.time_stamps.heartbeat = get_time_usec();
//					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
				break;
			}

			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
//                printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                mavlink_msg_global_position_int_decode(&message,&(currentMessages_.global_position_int));
                currentMessages_.lastGlobalPosition = get_usec();
//					current_messages.time_stamps.global_position_int = get_time_usec();
//					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
				break;
			}

			case MAVLINK_MSG_ID_GPS_RAW_INT: {
                addLogMessage(vl_DEBUG,port_ + ": received GPS message");
//					std::cout << port_ << ": MAVLINK MSG ID GPS RAW INT" << std::endl;
				//printf("MAVLINK_MSG_ID_GPS_RAW_INT\n");
                mavlink_msg_gps_raw_int_decode(&message,&(currentMessages_.gps_raw_int));
                currentMessages_.lastRawGpsPosition = get_usec();
				break;
			}

			case MAVLINK_MSG_ID_GPS_STATUS: {
				//printf("MAVLINK_MSG_ID_GPS_STATUS\n");
                mavlink_msg_gps_status_decode(&message,&(currentMessages_.gps_status));
                currentMessages_.lastGpsStatus = get_usec();
				break;
			}

			case MAVLINK_MSG_ID_HIGHRES_IMU: {
				//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                mavlink_msg_highres_imu_decode(&message,&(currentMessages_.highres_imu));
                currentMessages_.lastHighresImu = get_usec();
//					current_messages.time_stamps.highres_imu = get_time_usec();
//					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
				break;
			}

			case MAVLINK_MSG_ID_ATTITUDE: {
				//printf("MAVLINK_MSG_ID_ATTITUDE\n");
                mavlink_msg_attitude_decode(&message,&(currentMessages_.attitude));
                currentMessages_.lastAttitude = get_usec();
//					current_messages.time_stamps.attitude = get_time_usec();
//					this_timestamps.attitude = current_messages.time_stamps.attitude;
				break;
			}

			case MAVLINK_MSG_ID_RADIO_STATUS: {
//                printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
                mavlink_msg_radio_status_decode(&message,&(currentMessages_.radio_status));
                currentMessages_.lastRadioStatus = get_usec();
//					current_messages.time_stamps.radio_status = get_time_usec();
//					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
				break;
			}

			default: {
				// printf("Warning, did not handle message id %i\n",message.msgid);
				break;
			}

			} // end: switch msgid
			pthread_mutex_unlock(&messagesLock_);

		} // end: if read message

	} // end: while keepReadingMavlink_

	return;
}
bool TSmavlinkReader::start() {
	if (initialized_ != true) {
		addLogMessage(vl_ERROR, "mavlinkReader not properly initialized");
		return false;
	}
	// std::cout << "starting thread reading MAVLINK messages";
	if (readingStatus_ != false) {
		// printf(stderr,"read thread already running\n");
		// TODO: add some logging here
		return false;
	}

	if (!openSerial()) {
		addLogMessage(vl_ERROR, "Could not open serial port " + port_);
	} else {
		// open serial port
		pthread_create(&readThreadId_, NULL, readThread, (void*) this);
		addLogMessage(vl_INFO, "Started thread reading MAVLINK messages");
		return true;
	}

	return false;

}

bool TSmavlinkReader::openSerial() {

	fd_ = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NDELAY);

	// check if open was successful
	if (fd_ == -1) {
		// could not open the port
		addLogMessage(vl_ERROR,
				"Could not open serial device on port " + port_);
		return false;
	} else {
		fcntl(fd_, F_SETFL, 0);
	}

	bool configSucessfull = setupSerial();
	if (!configSucessfull) {
		addLogMessage(vl_ERROR,
				"Could not configure serial device on port " + port_);
		// could not configure port
		return false;
	}

	lastStatus_.packet_rx_drop_count = 0;

	isOpen_ = true;

	addLogMessage(vl_INFO,
			"Serial device on " + port_ + " successfully opened.");

	// Done!
	return true;

}

bool TSmavlinkReader::closeSerial() {

	// TODO: add some logging
	int result = close(fd_);

	if (result) {
		// add some more logging here
		return false;
	}

	isOpen_ = false;
	return true;
}

bool TSmavlinkReader::setupSerial() {
	if (!isatty(fd_)) {
		// TODO: add some logging
		return false;
	}

	// Read file descriptor configuration
	struct termios config;
	if (tcgetattr(fd_, &config) < 0) {
		// TODO: add some logging
		return false;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN] = 1;
	config.c_cc[VTIME] = 10; // was 0

	switch (baudrate_) {
	case 57600:
		if (cfsetispeed(&config, B57600) < 0
				|| cfsetospeed(&config, B57600) < 0) {
			// TODO: add some logging
			return false;
		}
		break;
	case 115200:
		if (cfsetispeed(&config, B115200) < 0
				|| cfsetospeed(&config, B115200) < 0) {
			// TODO: add some logging
			return false;
		}
		break;

	default:
		// TODO: add some logging
		return false;
		break;
	}

	// Finally, apply the configuration
	if (tcsetattr(fd_, TCSAFLUSH, &config) < 0) {
		// fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd_);
		// TODO: add some logging
		return false;
	}
	return true;
}

void TSmavlinkReader::stopReading() {
	keepReadingMavlink_ = false;

	pthread_join(readThreadId_, NULL);
}

void TSmavlinkReader::getMavlinkMessages(MavlinkMessages& messages) {
	pthread_mutex_lock(&messagesLock_);
	messages = currentMessages_;
	pthread_mutex_unlock(&messagesLock_);
}

void* TSmavlinkReader::readThread(void* arg) {
	TSmavlinkReader *mavlinkReader = (TSmavlinkReader *) arg;

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

bool TSmavlinkReader::keepReading() {
	return keepReadingMavlink_;
}

uint64_t TSmavlinkReader::get_usec() {
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

} /* namespace tracking */
