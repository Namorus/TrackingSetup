/*
 * TATrackingMode.h
 *
 *  Created on: Jun 17, 2013
 *      Author: thomas
 */

#ifndef TATRACKINGMODE_H_
#define TATRACKINGMODE_H_

#include <trackingsetup/config.h>
#include <trackingsetup/trackingsetup.h>

namespace tracking {

class TrackingMode: public tracking::TrackingSetup {
public:
	void setConfig(Config* pcfg);
//	setpoint getNewSetpoint();

	setpoints getNewSetpoints();

protected:
	Config cfg;

//	setpoint newSetpoint;

	setpoints newSetpoints;

	void setNewSetpoints(setpoints newSetpoints);

	void setNewSetpoints(TActrlType type, double pan, double tilt);

	void setNewSetpoints(TActrlType panType, double panValue,
			TActrlType tiltType, double tiltValue);

	void setNewPanSetpoint(TActrlType type, double _value);
	void setNewTiltSetpoint(TActrlType type, double _value);
};

} /* namespace tracking */
#endif /* TATRACKINGMODE_H_ */
