/*
 * TATrackingMode.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: thomas
 */

#include <trackingsetup/tracking_mode.h>

namespace tracking {

void TrackingMode::setConfig(Config* pcfg) {
	cfg = *pcfg;

}

setpoints TrackingMode::getNewSetpoints() {
	setpoints temp = newSetpoints;
	newSetpoints.panCtrltype = ct_undefined;
	newSetpoints.panValue = 0;
	newSetpoints.tiltCtrltype = ct_undefined;
	newSetpoints.tiltValue = 0;
	return temp;
}

void TrackingMode::setNewSetpoints(setpoints _newSetpoints) {
	newSetpoints = _newSetpoints;
}

void TrackingMode::setNewSetpoints(TActrlType type, double pan, double tilt) {
	newSetpoints.panCtrltype = type;
	newSetpoints.panValue = pan;
	newSetpoints.tiltCtrltype = type;
	newSetpoints.tiltValue = tilt;
}

void TrackingMode::setNewSetpoints(TActrlType panType, double panValue,
		TActrlType tiltType, double tiltValue) {
	newSetpoints.panCtrltype = panType;
	newSetpoints.panValue = panValue;
	newSetpoints.tiltCtrltype = tiltType;
	newSetpoints.tiltValue = tiltValue;
}

void TrackingMode::setNewPanSetpoint(TActrlType type, double _value) {
	newSetpoints.panCtrltype = type;
	newSetpoints.panValue = _value;
}
void TrackingMode::setNewTiltSetpoint(TActrlType type, double _value) {
	newSetpoints.tiltCtrltype = type;
	newSetpoints.tiltValue = _value;
}

} /* namespace tracking */
