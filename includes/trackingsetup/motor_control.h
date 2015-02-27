/*
 * TAMotorControl.h
 *
 *  Created on: Jun 17, 2013
 *      Author: thomas
 */

#ifndef TAMOTORCONTROL_H_
#define TAMOTORCONTROL_H_

#include <trackingsetup/epos.h>
#include <trackingsetup/trackingsetup.h>

namespace tracking {

class MotorControl: public TrackingSetup {
public:
	MotorControl();
	~MotorControl();

	void init(MotorControlConf* pcfg);

	void enablePan();

	void enableTilt();

	// returns current tilt angle (in degrees)
	double getTiltAngle();

	// return current pan angle (in degrees)
	double getPanAngle();

	// returns current tilt position Epos coordinates [qc]
	int getTiltPosition();

	// return current pan angle Epos coordinates [qc]
	int getPanPosition();

	// stop all motors and disable them
	int stop();

	// perform quickstop
	void quickstop();

    void displayDigitalInputState();

	/*!
	 * Sets new pan velocity (is directly sent to controller)
	 * @param newPanSpeed Velocity in deg/s
	 */
	void setPanVelocity(float newPanSpeed);
	float getPanVelocityMust();

	void setTiltVelocity(float newTiltSpeed);
	float getTiltVelocityMust();

	bool panPositionReached();

	bool tiltPositionReached();

	void setPanAngle(double panAngleMust);

	void setTiltAngle(double tiltAngleMust);

	void setPanPosition(int panPositionMust);

	void setTiltPosition(int panPositionMust);

	void setRelativePanAngle(double relPanAngleMust);

	void setRelativeTiltAngle(double relTiltAngleMust);

	void fetchData();

	/*!
	 * get current tilt speed
	 * @return float with tilt velocity in [deg/s]
	 */
	float getTiltSpeed();

	/*!
	 * get current pan speed
	 * @return float with pan velocity in [deg/s]
	 */
	float getPanSpeed();

	// initialize tiltMotor:
	// set 0 degree (homing)
	int homeTiltMotor();

	bool tiltMotorIsHoming();

	bool isTiltHomed();

	/* some helper functions */

	/*!
	 * compute epos position equivalent of angle in degree (using fixed gearRatio)
	 * assuming 0 = f(0)
	 * !! use with caution, this function is injective non-surjective,
	 * !!   since deg = [0, 360), qc = [-2^31, 2^31 - 1]
	 * @param deg
	 * @return equivalent of epos position [qc]
	 */
	static int deg2epos(double deg);

	/*!
	 * compute angle equivalent of epos position  (using fixed gearRatio)
	 * assuming 0 = f(0)
	 * !! use with caution, this function is non-injective surjective  (n to 1)
	 * @param qc
	 * @return
	 */
	static float epos2deg(int qc);

	// velocity [deg/s] to motor_revs / min
	static int deg_s2epos(double deg_s);

	// motor_revs / min to velocity [deg/s];
	static float epos2deg_s(int epos);

	template<class T>
	static T getMin(T a, T b) {
		return a > b ? a : b;
	}

	bool isInitialized();

	bool commOK();

	friend std::ostream& operator<<(std::ostream& out,
			const MotorControl& motorControl);

private:
	EposComm* pEposPan;
	EposComm* pEposTilt;

	Epos* panController;
	Epos* tiltController;

	static const float gearRatio = 12167.0 / 64.0;

	float tiltVelocityMust;
	float panVelocityMust;

	struct curMotorData {
		curMotorData() :
				panAngle(0), tiltAngle(0), panPosition(0), tiltPosition(0), panVelocity(
						0), tiltVelocity(0) {
		}
		float panAngle;
		float tiltAngle;
		int panPosition;
		int tiltPosition;
		float panVelocity;
		float tiltVelocity;
	} curData;

	bool initialized;
//	bool tiltIsHomed;
	bool tiltIsHoming;

	int commOpenRes;

	MotorControlConf* pconfig;

};

} /* namespace tracking */
#endif /* TAMOTORCONTROL_H_ */
