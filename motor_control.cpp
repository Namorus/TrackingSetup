/*
 * TAMotorControl.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: thomas
 */

#include <trackingsetup/motor_control.h>

namespace tracking {
MotorControl::MotorControl() :
		pEposPan(NULL), pEposTilt(NULL), panController(NULL), tiltController(
				NULL), tiltVelocityMust(0.0), panVelocityMust(0.0), initialized(
				false),
//	tiltIsHomed (false),
		tiltIsHoming(false), commOpenRes(0), pconfig(NULL) {

//	tiltController->setVerbose(true);
}

MotorControl::~MotorControl() {
	pEposPan->close();
	pEposTilt->close();
}

void MotorControl::init(MotorControlConf* arg) {
	if (arg != NULL) {
		pconfig = arg;
		// using devices serialnumber for identification (pan / tilt)
		pEposPan = new EposComm(pconfig->panEposNo.c_str(),
				pconfig->commRetries, 0);
		pEposTilt = new EposComm(pconfig->tiltEposNo.c_str(),
				pconfig->commRetries, 0);

		commOpenRes += pEposPan->open();
		commOpenRes += pEposTilt->open();

		panController = new Epos(pEposPan, 0, "pan");
		tiltController = new Epos(pEposTilt, 0, "tilt");
		addLogChild(panController);
		addLogChild(tiltController);

		if (panController->getCurrentOperationMode() != Epos::OMD_CURRENT_MODE
				&& panController->getCurrentOperationMode()
						!= Epos::OMD_HOMING_MODE
				&& panController->getCurrentOperationMode()
						!= Epos::OMD_INTERPOLATED_POSITION_MODE
				&& panController->getCurrentOperationMode()
						!= Epos::OMD_MASTER_ENCODER_MODE
				&& panController->getCurrentOperationMode()
						!= Epos::OMD_POSITION_MODE
				&& panController->getCurrentOperationMode()
						!= Epos::OMD_PROFILE_POSITION_MODE
				&& panController->getCurrentOperationMode()
						!= Epos::OMD_PROFILE_VELOCITY_MODE
				&& panController->getCurrentOperationMode()
						!= Epos::OMD_STEP_DIRECTION_MODE
				&& panController->getCurrentOperationMode()
						!= Epos::OMD_VELOCITY_MODE) {
			commOpenRes = -99;
		}
	}

	// setting up controllers
	panController->clearFault();

//	panController->setMinimalPositionLimit(deg2epos(-180.0));
//	panController->setMaximalPositionLimit(deg2epos(180));
	panController->disablePositionLimits();
	panController->setMaxProfileVelocity(6000);
	panController->setMaxAcceleration(5000);
	panController->setProfileVelocity(6000);
    panController->setProfileAcceleration(2000); // 5000
    panController->setProfileDeceleration(2000); // 5000
    panController->setQuickStopDeceleration(5000); // 10000
	panController->setPositionWindow(1000);
    panController->setDigitialIn1ToDeviceEnable();
	panController->enable();

//	panController->enableMotor(CEpos2::PROFILE_POSITION);

	tiltController->clearFault();
	tiltController->setMinimalPositionLimit(deg2epos(-20.0));
	tiltController->setMaximalPositionLimit(deg2epos(200.0));
	tiltController->setMaxProfileVelocity(7600);
	tiltController->setProfileVelocity(7600);
	tiltController->setPositionWindow(1000);
    tiltController->setProfileAcceleration(2000);
    tiltController->setProfileDeceleration(2000);
    tiltController->setQuickStopDeceleration(5000);
    tiltController->setDigitialIn1ToDeviceEnable();
    tiltController->setDigitialIn4ToHomeSwitch();
	tiltController->enable();
//	tiltController->enableMotor(CEpos2::PROFILE_POSITION);

	initialized = true;

}

void MotorControl::enablePan() {
	panController->enable();
}

void MotorControl::enableTilt() {
	tiltController->enable();
}

bool MotorControl::tiltMotorIsHoming() {
	return tiltIsHoming;
}

bool MotorControl::commOK() {
	return (bool) (commOpenRes == 0);
}

int MotorControl::stop() {

	panController->setState(Epos::DISABLE_OPERATION);
	tiltController->setState(Epos::DISABLE_OPERATION);

	return 0;
}

void MotorControl::setTiltVelocity(float newTiltSpeed) {

//	tiltController->enable();
	tiltController->setOperationMode(Epos::OMD_PROFILE_VELOCITY_MODE);
	tiltController->setMotionProfileType(1);
	tiltController->setTargetVelocity(deg_s2epos(newTiltSpeed));
	tiltVelocityMust = newTiltSpeed;
	tiltController->startProfileVelocity();

}

float MotorControl::getTiltVelocityMust() {
	return tiltVelocityMust;
}

bool MotorControl::isInitialized() {
	return initialized;
}

void MotorControl::quickstop() {
	tiltController->quickStop();
	panController->quickStop();
	this->tiltVelocityMust = 0;
	this->panVelocityMust = 0;
}

float MotorControl::getTiltSpeed() {
//	return tiltController->getActualVelocity();
	return curData.tiltVelocity;

}

float MotorControl::getPanSpeed() {
//	return panController->getActualVelocity();
	return curData.panVelocity;
}

void MotorControl::setPanVelocity(float newPanSpeed) {
//	panController->enable();
	panController->setOperationMode(Epos::OMD_PROFILE_VELOCITY_MODE);
	panController->setMotionProfileType(1);
	panController->setTargetVelocity(deg_s2epos(newPanSpeed));
	panVelocityMust = newPanSpeed;
	panController->startProfileVelocity();
}

float MotorControl::getPanVelocityMust() {
	return panVelocityMust;
}

void MotorControl::setRelativePanAngle(double relPanAngleMust) {
	panController->setOperationMode(Epos::OMD_PROFILE_POSITION_MODE);
	panController->setMotionProfileType(1);
	panController->setTargetPosition(deg2epos(relPanAngleMust));
	panController->startRelativeMotion();
}

void MotorControl::setPanAngle(double panAngleMust) {
	// read out current operation mode for later resetting.
//	Epos::operational_mode_t curOpMode = panController->getActualOperationMode();

// ensure that 0<= panAngleMust < 360;
	if (panAngleMust < 0) {
		panAngleMust = fmod(panAngleMust, 360.0) + 360;
	} else if (panAngleMust >= 360) {
		panAngleMust = fmod(panAngleMust, 360.0);
	}

	// set controller in to possitoin mode
	panController->setOperationMode(Epos::OMD_PROFILE_POSITION_MODE);

	// use sin^2 Profile (smoother)
	panController->setMotionProfileType(1);

	// get current angle
	double curAngle = this->getPanAngle();

	// rotate clockwise or ccw ?
	double delta = panAngleMust - curAngle;

	if (delta <= 180 && delta > -180) {
		panController->setTargetPosition(deg2epos(delta));
	} else if (delta < -180) {
		panController->setTargetPosition(deg2epos(delta + 360.0));
	} else if (delta > 180) {
		panController->setTargetPosition(deg2epos(delta - 360.0));
	}

	panController->startRelativeMotion();

	// set previous operation mode
//	panController->setOperationMode(curOpMode);
}

void MotorControl::setPanPosition(int panPosMust) {
	// read out current operation mode for later resetting.
//	Epos::operational_mode_t curOpMode = panController->getActualOperationMode();

	panController->setOperationMode(Epos::OMD_PROFILE_POSITION_MODE);
	panController->setMotionProfileType(1);
	panController->setTargetPosition(panPosMust);
	panController->startAbsoluteMotion();

	// set previous operation mode
//	panController->setOperationMode(curOpMode);
}

bool MotorControl::setTiltAngle(double tiltAngleMust) {
	// read out current operation mode for later resetting.
//	Epos::operational_mode_t curOpMode = tiltController->getActualOperationMode();

	int result = true;
//	tiltController->stopProfileVelocity();

	tiltController->setOperationMode(Epos::OMD_PROFILE_POSITION_MODE);

	if(!tiltController->setMotionProfileType(1)) {
		result = false;
		addLogMessage(vl_WARNING,"TiltMotorControl :: Could not set motion profile to sinusoidal!");
	}

	if (!tiltController->setTargetPosition(deg2epos(tiltAngleMust))) {
		result = false;
		addLogMessage(vl_WARNING,"TiltMotorControl :: Could not set target position!");
	}
	usleep(1000);

	if (!tiltController->setControlword(0x003f)) {
		result = false;
		addLogMessage(vl_WARNING,"TiltMotorControl :: Error starting absolute motion!");
	}

	return result;

	// set previous operation mode
//	tiltController->setOperationMode(curOpMode);
}

void MotorControl::setTiltPosition(int tiltPosMust) {
	// read out current operation mode for later resetting.
//	Epos::operational_mode_t curOpMode = tiltController->getActualOperationMode();

	tiltController->setOperationMode(Epos::OMD_PROFILE_POSITION_MODE);
	tiltController->setMotionProfileType(1);
	tiltController->setTargetPosition(tiltPosMust);
	tiltController->startAbsoluteMotion();

	// set previous operation mode
//	tiltController->setOperationMode(curOpMode);
}
void MotorControl::setRelativeTiltAngle(double relTiltAngleMust) {
	tiltController->setOperationMode(Epos::OMD_PROFILE_POSITION_MODE);
	tiltController->setMotionProfileType(1);
	tiltController->setTargetPosition(deg2epos(relTiltAngleMust));
	tiltController->startRelativeMotion();
}
bool MotorControl::isTiltHomed() {
	return tiltController->isHomingAttained();
}

int MotorControl::getTiltPosition() {
//	return tiltController->getActualPosition();
	return curData.tiltPosition;
}

double MotorControl::getTiltAngle() {
//	return epos2deg(tiltController->getActualPosition());
	return curData.tiltAngle;
}

int MotorControl::homeTiltMotor() {

	// set operation mode to homing mode
	tiltController->setOperationMode(Epos::OMD_HOMING_MODE);

	// configure homing
//    tiltController->setSwitchHoming(500, 200, 500, -deg2epos(0));
    tiltController->setIndexHoming(500, 200, 500, -deg2epos(90));

	// do homing
	tiltController->startHoming();
	tiltIsHoming = true;
	return 0;
}

bool MotorControl::panPositionReached() {
	return (bool) panController->isTargetReached();
}

bool MotorControl::tiltPositionReached() {
	return (bool) tiltController->isTargetReached();
}

int MotorControl::getPanPosition() {
//	return panController->getActualPosition();
	return curData.panPosition;
}

double MotorControl::getPanAngle() {
//	double curPanAngle = fmod(epos2deg(getPanPosition()),360.0);
//	if (curPanAngle < 0)
//		curPanAngle += 360.0;
//
//	return curPanAngle;
	return curData.panAngle;
}

void MotorControl::displayDigitalInputState() {
    std::cout << "Pan: " << panController->getCurrentDigitalInputState() << "\t";
    std::cout << "Tilt: " << tiltController->getCurrentDigitalInputState() << std::endl;
}

int MotorControl::deg2epos(double deg) {
	return (int) (((deg * gearRatio * 2000.0) / 360) + 0.5);
}

int MotorControl::deg_s2epos(double deg_s) {

	// velocity = rev/min = deg/s * 60s/min * 1rev/360deg * gearRatio
	return (int) (deg_s / 6.0 * gearRatio + 0.5);
}

float MotorControl::epos2deg_s(int epos) {

	if (gearRatio != 0) {
		return 6.0 * epos / gearRatio;
	}

	return 0;
}

float MotorControl::epos2deg(int qc) {

	if (gearRatio != 0) {
		return (float) qc / gearRatio * 360.0 / 2000.0;
	}

	return 0;
}

void MotorControl::fetchData() {
	curData.panPosition = panController->getActualPosition();
	curData.tiltPosition = tiltController->getActualPosition();

	curData.panAngle = fmod(epos2deg(curData.panPosition), 360.0);
	if (curData.panAngle < 0) {
		curData.panAngle += 360.0;
	}
	curData.tiltAngle = epos2deg(curData.tiltPosition);

	curData.panVelocity = panController->getActualVelocity();
	curData.tiltVelocity = tiltController->getActualVelocity();
}


std::ostream& operator<<(std::ostream& out,
		const MotorControl& motorControl) {
	out << "$MotorData ";
	out.precision(6);
	out << motorControl.curData.panPosition << " "
			<< motorControl.curData.tiltPosition << " "
			<< motorControl.curData.panAngle << " "
			<< motorControl.curData.tiltAngle << " "
			<< MotorControl::epos2deg_s(motorControl.curData.panVelocity)
			<< " "
			<< MotorControl::epos2deg_s(motorControl.curData.tiltVelocity);
	return out;
}

} /* namespace tracking */
