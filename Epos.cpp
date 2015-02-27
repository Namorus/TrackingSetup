/*
 * Epos.cpp
 *
 *  Created on: May 23, 2013
 *  Last change on: February 27, 2015
 *      Author: Thomas Mantel
 */

#include "trackingsetup/Epos.h"

namespace tracking {

Epos::Epos(EposComm* _pGateway, uint8_t _nodeId, const std::string _deviceName) :
		pGateway(_pGateway), nodeId(_nodeId), deviceName(_deviceName) {
	OpMode = getActualOperationMode();

}

const std::string Epos::getDeviceName() const {
	return deviceName;
}

int32_t Epos::ReadObjectValue(uint16_t index, uint8_t subindex) {

	uint16_t ans_frame[8];
	int r = 0;
	for (int attempts = 0; attempts < 5 && r == 0; attempts++) {
		r = pGateway->ReadObject(ans_frame, 8, index, subindex, this->nodeId);
	}
	if (r == 0) {
		std::cout << "Error while reading from EPOS \"" << this->deviceName
				<< "\": " << index << ", " << subindex << std::endl;
		return 0;
	}

	int32_t result = (ans_frame[4] << 16) | ans_frame[3];

	return result;
}

bool Epos::WriteObjectValue(uint16_t index, uint8_t subindex, uint32_t data) {
	return pGateway->WriteObject(index, subindex, data, this->nodeId);
}

/*! check EPOS state, firmware spec 8.1.1

 \return EPOS status as defined in firmware specification 8.1.1

 */
Epos::actual_state_t Epos::getState() {
	uint16_t w = getStatusWord();

	//printEPOSstatusword(w);

	return status2state(w);
}

/* read EPOS status word */
uint16_t Epos::getStatusWord() {
	return ReadObjectValue(0x6041, 0x00);
}

Epos::actual_state_t Epos::status2state(uint16_t w) {
	/* state 'start' (0)
	 fedc ba98  7654 3210
	 w == x0xx xxx0  x000 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && !bitcmp(w, E_BIT08)
			&& !bitcmp(w, E_BIT14))
		return (START);

	/* state 'not ready to switch on' (1)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (NOT_READY_TO_SWITCH_ON);

	/* state 'switch on disabled' (2)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x100 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (SWITCH_ON_DISABLED);

	/* state 'ready to switch on' (3)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x010 0001 */
	if (bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (READY_TO_SWITCH_ON);

	/* state 'switched on' (4)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x010 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (SWITCHED_ON);

	/* state 'refresh' (5)
	 fedc ba98  7654 3210
	 w == x1xx xxx1  x010 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && bitcmp(w, E_BIT14))
		return (REFRESH);

	/* state 'measure init' (6)
	 fedc ba98  7654 3210
	 w == x1xx xxx1  x011 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && bitcmp(w, E_BIT14))
		return (MEASURE_INIT);

	/* state 'operation enable' (7)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x011 0111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (OPERATION_ENABLE);

	/* state 'quick stop active' (8)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x001 0111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (QUICK_STOP_ACTIVE);

	/* state 'fault reaction active (disabled)' (9)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 1111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (FAULT_REACTION_ACTIVE_DISABLED);

	/* state 'fault reaction active (enabled)' (10)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x001 1111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (FAULT_REACTION_ACTIVE_ENABLED);

	/* state 'fault' (11)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 1000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (FAULT);

	// if we get down here, statusword has a unknown value!
	fprintf(stderr, "WARNING: EPOS status word %#06x is an unknown state!\n",
			w);

	return (UNKNOWN);
}

/* pretty-print EPOS state */
int Epos::printState() {
	actual_state_t state = getState();

//	std::cout << "EPOS node " << getDeviceName() << ": is in state: ";

	switch (state) {
	case START:
		std::cout << "Start\n";
		std::cout << "\tBootup\n";
		break;
	case NOT_READY_TO_SWITCH_ON:
		std::cout << "Not Ready to Switch On\n";
		std::cout << "\tCurrent offset will be measured\n";
		std::cout << "\tDrive function is disabled\n";
		break;
	case SWITCH_ON_DISABLED:
		std::cout << "Switch On Disabled\n";
		std::cout << "\tDrive initialization is complete\n";
		std::cout << "\tDrive parameters may be changed\n";
		std::cout << "\tDrive function is disabled\n";
		break;
	case READY_TO_SWITCH_ON:
		std::cout << "Ready to Switch On\n";
		std::cout << "\tDrive parameters may be changed\n";
		std::cout << "\tDrive function is disabled\n";
		break;
	case SWITCHED_ON:
		std::cout << "Switched On\n";
		std::cout << "\tDrive function is disabled\n";
		break;
	case REFRESH:
		std::cout << "Refresh\n";
		std::cout << "\tRefresh of power stage\n";
		break;
	case MEASURE_INIT:
		std::cout << "Measure Init\n";
		std::cout << "\tPower is applied to the motor\n";
		std::cout << "\tMotor resistance or commutation delay is measured\n";
		break;
	case OPERATION_ENABLE:
		std::cout << "Operation Enable\n";
		std::cout << "\tNo faults have been detected\n";
		std::cout
				<< "\tDrive function is enabled and power is applied to the motor\n";
		break;
	case QUICK_STOP_ACTIVE:
		std::cout << "Quickstop Active\n";
		std::cout << "\tQuickstop function is being executed\n";
		std::cout
				<< "\tDrive function is enabled and power is applied to the motor\n";
		break;
	case FAULT_REACTION_ACTIVE_DISABLED:
		std::cout << "Fault Reaction Active (disabled)\n";
		std::cout << "\tA fault has occurred in the drive\n";
		std::cout << "\tDrive function is disabled\n";
		break;
	case FAULT_REACTION_ACTIVE_ENABLED:
		std::cout << "Fault Reaction Active (enabled)\n";
		std::cout << "\tA fault has occurred in the drive\n";
		std::cout << "\tSelected fault reaction is being executed\n";
		break;
	case FAULT:
		std::cout << "FAULT\n";
		std::cout << "\tA fault has occurred in the drive\n";
		std::cout << "\tDrive parameters may be changed\n";
		std::cout << "\tDrive function is disabled\n";
		break;

	default:
		std::cout << "UNKNOWN!\n";
		return (-1);
	}
	return (0);
}

void Epos::enable() {
	// TODO: handle initial error conditions
	actual_state_t state = getState();

//	std::cout << "EPOS node " << getDeviceName()
//			<< ": resetting from state '" << stateDescription(state) << "'"
//			<< std::endl;

// FAULT
	if (state == FAULT) {
//		UNSIGNED8 errReg = getErrorRegister();
//		if (errReg) {
//			std::cout << "printErrorRegister() = 0x" << std::hex << (int) errReg << std::endl;
//			printErrorRegister(errReg);

//			uint8_t errNum = getNumberOfErrors();
//			std::cout << "Number of Errors = " << (unsigned int) errNum << std::endl;

//			for (int i = 1; i < errNum + 1; ++i) {
//				uint32_t errCode = getErrorHistory(i);
//				std::cout << "Error at index " << i << " is " <<
//						std::hex << (int) errCode << ": " <<
//						ErrorCodeMessage(errCode) << std::endl;
//			}
//		}

// We are not supposed to clear faults here
//		BOOST_THROW_EXCEPTION(fe() << reason("Device is in the fault state"));
	} else if (state != OPERATION_ENABLE) {
		// Shutdown
		setState(SHUTDOWN);

		// Setup the wakeup time
//		boost::system_time wakeup = boost::get_system_time();

		int retry = 5;
		do {
			state = getState();

			if (state == READY_TO_SWITCH_ON) {
				break;
			} else if (state == QUICK_STOP_ACTIVE) {
				break;
			} else if (state == FAULT) {
//				BOOST_THROW_EXCEPTION(fe() << reason("Device is in the fault state"));
			} else {
//				std::cout << "EPOS node " << getDeviceName() << ": transited to state '" << stateDescription(state)
//						<< "' during shutdown" << std::endl;
				// Continue;
			}

			// Increment the wakeup time
//			wakeup += boost::posix_time::milliseconds(5);

			// Wait for device state to change
//			boost::thread::sleep(wakeup);
			usleep(1000);

		} while (--retry);

		if (retry == 0) {
//			BOOST_THROW_EXCEPTION(fe() << reason("Timeout shutting device down"));
		}

		// Ready-to-switch-On expected
		if (state != READY_TO_SWITCH_ON && state != QUICK_STOP_ACTIVE) {
//			BOOST_THROW_EXCEPTION(fe() << reason("Ready-to-switch-On or Quick-Stop-Active expected"));
		}

		// Enable
		setState(ENABLE_OPERATION);

		// Setup the wakeup time
//		wakeup = boost::get_system_time();

		// Setup retry counter
		retry = 25;
		do {
			state = getState();

			// Condition to monitor for
			bool in_operation_enable = false;

			switch (state) {
			// These are expected transition states
			case SWITCHED_ON:
			case MEASURE_INIT:
			case REFRESH:
				break;
			case OPERATION_ENABLE:
				in_operation_enable = true;
				break;
			case FAULT:
//					BOOST_THROW_EXCEPTION(fe() << reason("Device is in the fault state"));
				break;
			default:
//					std::cout << "EPOS node " << getDeviceName() << ": transited to state '" << stateDescription(state)
//										<< "' during initialization" << std::endl;
				break;
			}

			// Exit loop if condition holds
			if (in_operation_enable)
				break;

			// Increment the wakeup time
//			wakeup += boost::posix_time::milliseconds(5);

			// Wait for device state to change
//			boost::thread::sleep(wakeup);

		} while (--retry);

//		if (retry == 0) {
//			BOOST_THROW_EXCEPTION(fe() << reason("Timeout enabling device"));
//		}
	}

	// Enable+Halt
	setControlword(0x010f);

	state = getState();

	// Operation Enabled expected
//	if (state != OPERATION_ENABLE) {
//		BOOST_THROW_EXCEPTION(fe() << reason("Operation Enable expected"));
//	}

//	std::cout << "EPOS node " << getDeviceName() << ": reset OK" << std::endl;
}

void Epos::clearFault(void) {
	// Print state.
//	   printState();
	// Check if node is in a FAULT state.
	if (getState() == FAULT) {
		/*
		 uint8_t errNum = getNumberOfErrors();
		 // Print list of errors.
		 for (UNSIGNED8 i = 1; i <= errNum; ++i) {
		 UNSIGNED32 errCode = getErrorHistory(i);
		 std::cout << "\t" << ErrorCodeMessage(errCode) << std::endl;
		 }
		 // Clear errors.
		 if (errNum > 0) {
		 clearNumberOfErrors();
		 }
		 */
		// Reset errors.
		setState(Epos::FAULT_RESET);

		// Recovery retry counter
		unsigned int retry = 0;

		// Recovery status flag
		bool recovered = false;

		// It takes some time to recovery from FAULT state
		while (retry++ < 5) {
			if (getState() == FAULT) {
				usleep(5000);
			} else {
				recovered = true;
				break;
			}
		}

	}
	// Reset node.
	// enable();
}

void Epos::setDigitialIn1ToDeviceEnable() {
    // set function of digital input 1 to 4: "Device Enable" (Firmware Spec. pp. 8-147)
    if (!WriteObjectValue(0x2070,0x01,4)) {
        addLogMessage(vl_WARNING,"Could not set function of digital input 1");
        return;
    }
    // Read the current "Digital Input Functionalities Execution Mask" and add Device Enable
    uint16_t difem = ReadObjectValue(0x2071,0x04);
    uint16_t difem_new = difem | 0x0010;
    if (!WriteObjectValue(0x2071,0x04, difem_new)) {
        addLogMessage(vl_WARNING,"Could not change digital input functionalities execution mask");
        return;
    }
    std::cout << "Current Execution Mask: " << ReadObjectValue(0x2071,0x04) << std::endl;

    // Read the current "Digital Input Functionalities Mask" and add Device Enable
    uint16_t difm = ReadObjectValue(0x2071,0x02);
    uint16_t difm_new = difm | 0x0010;
    if (!WriteObjectValue(0x2071,0x02, difm_new)) {
        addLogMessage(vl_WARNING,"Could not change \"Digital Input Functionalities Mask\"");
        return;
    }
}

void Epos::setDigitialIn4ToHomeSwitch() {
//    if (!WriteObjectValue(0x2070,0x04,2)) {
//        addLogMessage(vl_WARNING,"Could not set function of digital input 4");
//        return;
//    }

//    // Read the current "Digital Input Functionalities Execution Mask" and add Device Enable
//    uint16_t difem = ReadObjectValue(0x2071,0x04);
//    uint16_t difem_new = difem | 0x0004;
//    if (!WriteObjectValue(0x2071,0x04, difem_new)) {
//        addLogMessage(vl_WARNING,"Could not change digital input functionalities execution mask");
//        return;
//    }
//    std::cout << "Digital Input Functionalities Execution Mask: " << ReadObjectValue(0x2071,0x04) << std::endl;

    // Read the current "Digital Input Functionalities Mask" and add Device Enable
    uint16_t difm = ReadObjectValue(0x2071,0x02);
    uint16_t difm_new = difm | 0x0004;
    if (!WriteObjectValue(0x2071,0x02, difm_new)) {
        addLogMessage(vl_WARNING,"Could not change \"Digital Input Functionalities Mask\"");
        return;
    }
    std::cout << "Digital Input Functionalities Mask: " << ReadObjectValue(0x2071,0x02) << " (should be: " << difm_new << ")" << std::endl;

}


uint16_t Epos::getCurrentDigitalInputState() {
    return ReadObjectValue(0x2071,0x01);
}


/* change EPOS state according to firmware spec 8.1.3 */
void Epos::setState(desired_state_t state) {
	uint16_t cw = 0x0000;

	/* ! DO NOT READ OLD CONTROLWORD BACK, JUST SET THE BITS. It works
	 this way, but does NOT work otherways! -- mh, 07.07.06
	 */
	//cw = readControlword();
	switch (state) {
	case SHUTDOWN: // Shutdown: 0xxx x110
		cw &= ~E_BIT07;
		cw |= E_BIT02;
		cw |= E_BIT01;
		cw &= ~E_BIT00;
		setControlword(cw);
		break;
	case SWITCH_ON: // Switch On: 0xxx x111
		cw &= ~E_BIT07;
		cw |= E_BIT02;
		cw |= E_BIT01;
		cw |= E_BIT00;
		setControlword(cw);
		break;
	case SWITCH_ON_AND_ENABLE: // Switch On & Enable Operation: 0xxx 1111
		cw &= ~E_BIT07;
		cw |= E_BIT03;
		cw |= E_BIT02;
		cw |= E_BIT01;
		cw |= E_BIT00;
		setControlword(cw);
		break;
	case DISABLE_VOLTAGE: // Disable Voltage: 0xxx xx0x
		cw &= ~E_BIT07;
		cw &= ~E_BIT01;
		setControlword(cw);
		break;
	case QUICKSTOP: // Quickstop: 0xxx x01x
		cw &= ~E_BIT07;
		cw &= ~E_BIT02;
		cw |= E_BIT01;
		setControlword(cw);
		break;
	case DISABLE_OPERATION: // Disable Operation: 0xxx 0111
		cw &= ~E_BIT07;
		cw &= ~E_BIT03;
		cw |= E_BIT02;
		cw |= E_BIT01;
		cw |= E_BIT00;
		setControlword(cw);
		break;
	case ENABLE_OPERATION: // Enable Operation: 0xxx 1111
		cw &= ~E_BIT07;
		cw |= E_BIT03;
		cw |= E_BIT02;
		cw |= E_BIT01;
		cw |= E_BIT00;
		setControlword(cw);
		break;
	case FAULT_RESET: // Fault Reset 0xxx xxxx -> 1xxx xxxx
		cw |= E_BIT07;
		setControlword(0x0000);
		setControlword(cw);
		break;
	default:
//			BOOST_THROW_EXCEPTION(fe() << reason("ERROR: demanded state is UNKNOWN!"));
		// TODO: state
		break;
	}
}

/* write EPOS control word (firmware spec 14.1.57) */
void Epos::setControlword(uint16_t val) {
	WriteObjectValue(0x6040, 0x00, val);
}

/* set mode of operation --- 14.1.59 */
void Epos::setOperationMode(Epos::operational_mode_t m) {
	if (OpMode != m) {
		WriteObjectValue(0x6060, 0x00, (int32_t) m);

		OpMode = m;
	}
}

/* read mode of operation --- 14.1.60 */
Epos::operational_mode_t Epos::getActualOperationMode() {
	uint32_t mode = ReadObjectValue(0x6061, 0x00);
	return (operational_mode_t) mode;
}

void Epos::startAbsoluteMotion() {
	setControlword(0x003f);
}

void Epos::startRelativeMotion() {
	setControlword(0x005f);
}

/* compare WORD a with WORD b bitwise */
bool Epos::bitcmp(uint16_t a, uint16_t b) {
	return ((a & b) == b) ? true : false;
}

void Epos::moveRelative(int32_t steps) {
	// set the Profile Position Mode
	setOperationMode(OMD_PROFILE_POSITION_MODE);

	// write intended target position
	// firmware 14.1.70
	setTargetPosition(steps);

	// switch to relative positioning BY WRITING TO CONTROLWORD, finish	possible ongoing operation first!
	// see ->maxon applicattion note: device programming 2.1
	startRelativeMotion();
}

void Epos::moveAbsolute(int32_t steps) {
	// set the Profile Position Mode
	setOperationMode(OMD_PROFILE_POSITION_MODE);

	// write intended target position, is signed 32bit int
	// firmware 14.1.70
	setTargetPosition(steps);

	// switch to absolute positioning, cancel possible ongoing operation first!
	// see maxon application note: device programming 2.1
	startAbsoluteMotion();
}

/* read EPOS target position; firmware description 14.1.70 */
int32_t Epos::getTargetPosition() {
	return ReadObjectValue(0x607a, 0x00);
}

void Epos::setTargetPosition(int32_t val) {
	WriteObjectValue(0x607a, 0x00, val);
}

int32_t Epos::getDemandPosition() {
	return ReadObjectValue(0x6062, 0x00);
}

int32_t Epos::getActualPosition() {
	return ReadObjectValue(0x6064, 0x00);
}

int32_t Epos::getActualVelocity() {
	return ReadObjectValue(0x606c, 0x00);
}

uint32_t Epos::getPositionWindow() {
	return ReadObjectValue(0x6067, 0x00);

}

void Epos::setPositionWindow(uint32_t value) {
	WriteObjectValue(0x6067, 0x00, value);
}

int32_t Epos::getTargetVelocity() {
	return ReadObjectValue(0x60FF, 0x00);
}

void Epos::setTargetVelocity(int32_t val) {
	WriteObjectValue(0x60FF, 0x00, val);
}

void Epos::startProfileVelocity() {
	setControlword(0x000F);
}

void Epos::stopProfileVelocity() {
	setControlword(0x010F);
}

//void Epos::setVelocityModeSettingValue(int32_t val) {
//	WriteObjectValue(0x206B, 0x00, val);
//}

uint32_t Epos::getVelocityWindow() {

	return ReadObjectValue(0x606D, 0x00);
}

void Epos::setVelocityWindow(uint32_t velWindow) {

	WriteObjectValue(0x606D, 0x00, velWindow);
}

void Epos::setProfileVelocity(uint32_t vel) {
	WriteObjectValue(0x6081, 0x00, vel);
}

void Epos::setProfileAcceleration(uint32_t acc) {
	WriteObjectValue(0x6083, 0x00, acc);
}

void Epos::setProfileDeceleration(uint32_t dec) {
	WriteObjectValue(0x6084, 0x00, dec);
}

void Epos::setQuickStopDeceleration(uint32_t qsdec) {
	WriteObjectValue(0x6085, 0x00, qsdec);
}

void Epos::setMaxProfileVelocity(uint32_t maxvel) {
	WriteObjectValue(0x607F, 0x00, maxvel);
}

void Epos::setMaxAcceleration(uint32_t maxacc) {
	WriteObjectValue(0x60C5, 0x00, maxacc);
}

void Epos::setMotionProfileType(int16_t type) {
	WriteObjectValue(0x6086, 0x00, type);
}

bool Epos::isTargetReached() {
	return isTargetReached(getStatusWord());
}

bool Epos::isTargetReached(uint16_t status) {
	return (E_BIT10 & status);
}

/*! Read the Minimal Position Limit */
int32_t Epos::getMinimalPositionLimit() {
	return ReadObjectValue(0x607D, 0x01);
}

/*! Write the Minimal Position Limit */
void Epos::setMinimalPositionLimit(int32_t val) {
	WriteObjectValue(0x607D, 0x01, val);
}

/*! Read the Maximal Position Limit */
int32_t Epos::getMaximalPositionLimit() {
	return ReadObjectValue(0x607D, 0x02);
}

/*! Write the Maximal Position Limit */
void Epos::setMaximalPositionLimit(int32_t val) {
	WriteObjectValue(0x607D, 0x02, val);
}

void Epos::disablePositionLimits() {
	this->setMinimalPositionLimit(-0x80000000);
	this->setMaximalPositionLimit(+0x7FFFFFFF);
}

void Epos::quickStop() {
	setControlword(E_BIT05);
}

void Epos::setIndexHoming(int speed_pos, int speed_zero, int acc,
        int offset) {
    // use Homing method 7
    WriteObjectValue(0x6098, 0x00, (int8_t) 33);
    WriteObjectValue(0x6099, 0x01, (uint32_t) speed_pos);
    WriteObjectValue(0x6099, 0x02, (uint32_t) speed_zero);
    WriteObjectValue(0x609A, 0x00, (uint32_t) acc);
//    WriteObjectValue(0x2080, 0x00, (uint16_t) maxCurrent);
    WriteObjectValue(0x607C, 0x00, (int32_t) offset);
}

void Epos::setSwitchHoming(int speed_pos, int speed_zero, int acc,
        int offset) {
    // use Homing method 7
    WriteObjectValue(0x6098, 0x00, (int8_t) 7);
    WriteObjectValue(0x6099, 0x01, (uint32_t) speed_pos);
    WriteObjectValue(0x6099, 0x02, (uint32_t) speed_zero);
    WriteObjectValue(0x609A, 0x00, (uint32_t) acc);
//    WriteObjectValue(0x2080, 0x00, (uint16_t) maxCurrent);
    WriteObjectValue(0x607C, 0x00, (int32_t) offset);
}

void Epos::setMechanicalHoming(int speed_pos, int speed_zero, int acc,
		int offset, int maxCurrent) {
	// use Homing method -2
	WriteObjectValue(0x6098, 0x00, (int8_t) -2);
	WriteObjectValue(0x6099, 0x01, (uint32_t) speed_pos);
	WriteObjectValue(0x6099, 0x02, (uint32_t) speed_zero);
	WriteObjectValue(0x609A, 0x00, (uint32_t) acc);
	WriteObjectValue(0x2080, 0x00, (uint16_t) maxCurrent);
	WriteObjectValue(0x607C, 0x00, (int32_t) offset);
}

void Epos::setCurrentPosHoming(int speed_zero, int acc, int offset) {

	WriteObjectValue(0x6098, 0x00, (int8_t) 33);
	WriteObjectValue(0x6099, 0x02, (uint32_t) speed_zero);
	WriteObjectValue(0x609A, 0x00, (uint32_t) acc);
	WriteObjectValue(0x607C, 0x00, (int32_t) offset);

}

void Epos::startHoming() {
	OpMode = Epos::OMD_HOMING_MODE;
	setOperationMode(OpMode);

	setControlword(0x001F);
}

bool Epos::isHomingAttained() {

	bool homingAttained = false;
	if (OpMode == Epos::OMD_HOMING_MODE) {
		if (getStatusWord() & E_BIT12) {
			homingAttained = true;
		}
	}

	return homingAttained;
}

uint16_t Epos::getSpeedCtrlP() {
	return ReadObjectValue(0x60F9, 0x01);
}

void Epos::setSpeedCtrlP(uint16_t val) {
	WriteObjectValue(0x60F9, 0x01, val);
}

uint16_t Epos::getSpeedCtrlI() {
	return ReadObjectValue(0x60F9, 0x02);

}

void Epos::setSpeedCtrlI(uint16_t val) {
	WriteObjectValue(0x60F9, 0x02, val);

}

uint16_t Epos::getSpeedCtrlVelocityFF() {
	return ReadObjectValue(0x60F9, 0x04);

}

uint16_t Epos::getSpeedCtrlAccelFF() {
	return ReadObjectValue(0x60F9, 0x05);

}

} /* namespace tracking */

