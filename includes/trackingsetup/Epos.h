/*
 * Epos.h
 *
 *  Created on: May 23, 2013
 *  Last change on: February 27, 2015
 *      Author: Thomas Mantel
 */

#include <cstdio>
#include <iostream>
#include <stdint.h>

#include "trackingsetup/EposComm.h"
#include "trackingsetup/TAClass.h"
#include "trackingsetup/TAtypes.h"


namespace tracking {

#ifndef EPOS_H_
#define EPOS_H_

/* EPOS Statusword -- singe bits, see firmware spec 14.1.58 */
#define E_BIT15        0x8000      ///< bit code: position referenced to home position
#define E_BIT14        0x4000      ///< bit code: refresh cycle of power stage
#define E_BIT13        0x2000      ///< bit code: OpMode specific, some error
#define E_BIT12        0x1000      ///< bit code: OpMode specific
#define E_BIT11        0x0800      ///< bit code: NOT USED
#define E_BIT10        0x0400      ///< bit code: Target reached
#define E_BIT09        0x0200      ///< bit code: Remote (?)
#define E_BIT08        0x0100      ///< bit code: offset current measured (?)
#define E_BIT07        0x0080      ///< bit code: WARNING
#define E_BIT06        0x0040      ///< bit code: switch on disable
#define E_BIT05        0x0020      ///< bit code: quick stop
#define E_BIT04        0x0010      ///< bit code: voltage enabled
#define E_BIT03        0x0008      ///< bit code: FAULT
#define E_BIT02        0x0004      ///< bit code: operation enable
#define E_BIT01        0x0002      ///< bit code: switched on
#define E_BIT00        0x0001      ///< bit code: ready to switch on

class Epos : public TAClass {
private:
	EposComm* pGateway;
	uint8_t nodeId;

	std::string deviceName;

    int32_t ReadObjectValue(uint16_t index, uint8_t subindex);

    bool WriteObjectValue(uint16_t index, uint8_t subindex, uint32_t data);

	/*! \brief compare two 16bit bitmasks
	 *
	 * @return result of comparison */
	static bool bitcmp(uint16_t a, uint16_t b);

public:
//    int32_t ReadObjectValue(uint16_t index, uint8_t subindex);
//    bool WriteObjectValue(uint16_t index, uint8_t subindex, uint32_t data);

	Epos(EposComm* _pGateway, uint8_t _nodeId, const std::string _deviceName = std::string());

	//! Get device name
	const std::string getDeviceName() const;

	//! \brief Commands that can be send to EPOS controller in order to change its state.
	typedef enum _desired_state
	{
		SHUTDOWN, SWITCH_ON, SWITCH_ON_AND_ENABLE, DISABLE_VOLTAGE,
		QUICKSTOP, DISABLE_OPERATION, ENABLE_OPERATION, FAULT_RESET
	}desired_state_t;

	//! Actual state of the device
	typedef enum _actual_state_t {
		UNKNOWN = -1,
		START = 0,
		NOT_READY_TO_SWITCH_ON = 1,
		SWITCH_ON_DISABLED = 2,
		READY_TO_SWITCH_ON = 3,
		SWITCHED_ON = 4,
		REFRESH = 5,
		MEASURE_INIT = 6,
		OPERATION_ENABLE = 7,
		QUICK_STOP_ACTIVE = 8,
		FAULT_REACTION_ACTIVE_DISABLED = 9,
		FAULT_REACTION_ACTIVE_ENABLED = 10,
		FAULT = 11
	}actual_state_t;

	//! \brief check EPOS status
	//! @return state according to firmware spec
	actual_state_t getState();

	//! \brief Find EPOS state corresponding to given status word
	static actual_state_t status2state(uint16_t w);

	/*! \brief read Statusword */
	uint16_t getStatusWord();

	/*! \brief Prints pretty-formatted controller state.
	 *
	 * @retval 0 status is OK
	 * @retval -1 status is unknown
	 */
	int printState();

    uint16_t getCurrentDigitalInputState();

	//! \brief Enable the device by issuing a shutdown command followed by power-on and halt
	void enable();

	//! \brief High-level command to clear fault
	void clearFault();

    //! \brief Set Digital In 1 to Device Enable
    void setDigitialIn1ToDeviceEnable();
    //! \brief Set Digital In 4 to Home Switch (and enable it)
    void setDigitialIn4ToHomeSwitch();

	//! write EPOS control word
	void setControlword(uint16_t val);

	/*! \brief change EPOS state */
	void setState(desired_state_t state);

	/*! \brief read target position */
	int32_t getTargetPosition();

	/*! \brief read target position */
	void setTargetPosition(int32_t val);

	/*! \brief set OpMode to ProfilePosition and make absolute movement */
	void moveAbsolute(int32_t steps);

	/*! \brief set OpMode to ProfilePosition and make relative movement */
	void moveRelative(int32_t steps);

	//! \brief EPOS Operational mode
	typedef enum _operational_mode
	{
		OMD_INTERPOLATED_POSITION_MODE = 7,	//! interpolated position mode
		OMD_HOMING_MODE = 6,//! homing
		OMD_PROFILE_VELOCITY_MODE = 3,//! profile velocity mode
		OMD_PROFILE_POSITION_MODE = 1,//! profile position mode
		OMD_POSITION_MODE = -1,//! position mode
		OMD_VELOCITY_MODE = -2,//! velocity mode
		OMD_CURRENT_MODE = -3,//! current mode
		//OMD_DIAGNOSTIC_MODE = -4			//! diagnostic mode
		OMD_MASTER_ENCODER_MODE = -5,//! master encoder mode
		OMD_STEP_DIRECTION_MODE = -6//! step/direction mode
	}operational_mode_t;

	/*! \brief set EPOS mode of operation */
	void setOperationMode(operational_mode_t);

	/*! \brief read and returns  EPOS mode of operation
	 *
	 * @return 0 MEANS ERROR; '-1' is a valid OpMode, but 0 is not!
	 */
	operational_mode_t getActualOperationMode();

	//! \brief start motion with absolute demanded position
	void startAbsoluteMotion();

	//! \brief start motion with relative demanded position
	void startRelativeMotion();

	/*! \brief read demanded position */
	int32_t getDemandPosition();

	/*! \brief read actual position */
	int32_t getActualPosition();

	/*! \brief read actual velocity */
	int32_t getActualVelocity();

	/*! \brief read position window */
	uint32_t getPositionWindow();

	/*! \brief write position window */
	void setPositionWindow(uint32_t value);

	/*! \brief read position window time */
	//		int writePositionWindowTime(unsigned int val);
	//		int writePositionSoftwareLimits(long val, long val2);
	//! read velocity for velocity profile mode
	int32_t getTargetVelocity();

	//! write velocity for velocity profile mode
	void setTargetVelocity(int32_t val);

	void startProfileVelocity();

	void stopProfileVelocity();

	//! write velocity for velocity mode
	void setVelocityModeSettingValue(int32_t val);

	uint32_t getVelocityWindow();

	void setVelocityWindow(uint32_t velWindow);

	//! write velocity normally attained at the end of the acceleration ramp during a profiled move
	void setProfileVelocity(uint32_t vel);

	//! write acceleration ramp during a movement
	void setProfileAcceleration(uint32_t acc);

	//! write deceleration ramp during a movement
	void setProfileDeceleration(uint32_t dec);

	//! write deceleration ramp during a Quickstop
	void setQuickStopDeceleration(uint32_t qsdec);

	//! write maximal allowed speed
	void setMaxProfileVelocity(uint32_t maxvel);

	//! write maximal allowed acceleration
	void setMaxAcceleration(uint32_t maxacc);

	/*! \brief write position profile type
	 *
	 * @param type 0: linear ramp (trapezoidal profile), 1: sin^2 ramp (sinusoidal profile)
	 */
	void setMotionProfileType(int16_t type);

	//! \brief Is the movement target reached?
	bool isTargetReached();

	//! \brief Is the movement target reached?
	static bool isTargetReached(uint16_t status);

	/*! \brief read the Minimal Position Limit
	 * If the desired or the actual position is lower then the negative position
	 * limit a software position limit Error will be launched.
	 */
	int32_t getMinimalPositionLimit();

	/*! \brief write the Minimal Position Limit */
	void setMinimalPositionLimit(int32_t val);

	/*! \brief read the Maximal Position Limit */
	int32_t getMaximalPositionLimit();

	/*! \brief write the Maximal Position Limit */
	void setMaximalPositionLimit(int32_t val);

	/*! \brief disable position limits */
	void disablePositionLimits();

	void quickStop();

    void setIndexHoming(int speed_pos, int speed_zero, int acc, int offset);

    void setSwitchHoming(int speed_pos, int speed_zero, int acc, int offset);

	void setMechanicalHoming(int speed_pos, int speed_zero, int acc, int offset, int maxCurrent);

	void setCurrentPosHoming(int speed_zero, int acc, int offset = 0);

	void startHoming();

	bool isHomingAttained();

	uint16_t getSpeedCtrlP();

	void setSpeedCtrlP(uint16_t newpgain);

	uint16_t getSpeedCtrlI();

	void setSpeedCtrlI(uint16_t newigain);

	uint16_t getSpeedCtrlVelocityFF();

	uint16_t getSpeedCtrlAccelFF();

private:
	// needs to be at the bottom because of typedefs
	operational_mode_t OpMode;
};

#endif /* EPOS_H_ */

}
