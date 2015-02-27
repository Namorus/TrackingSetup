/*
 * TATrackingMode.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: thomas
 */

//#include "TAincludes.h"
#include "TATrackingMode.h"

namespace tracking {

void TATrackingMode::setConfig(TAConfig* pcfg) {
	cfg = *pcfg;

}

setpoints TATrackingMode::getNewSetpoints() {
	setpoints temp = newSetpoints;
	newSetpoints.panCtrltype = ct_undefined;
	newSetpoints.panValue = 0;
	newSetpoints.tiltCtrltype = ct_undefined;
	newSetpoints.tiltValue = 0;
	return temp;
}

void TATrackingMode::setNewSetpoints(setpoints _newSetpoints) {
	newSetpoints = _newSetpoints;
}

void TATrackingMode::setNewSetpoints(TActrlType type, double pan, double tilt) {
	newSetpoints.panCtrltype = type;
	newSetpoints.panValue = pan;
	newSetpoints.tiltCtrltype = type;
	newSetpoints.tiltValue = tilt;
}

void TATrackingMode::setNewSetpoints(TActrlType panType, double panValue,
		TActrlType tiltType, double tiltValue) {
	newSetpoints.panCtrltype = panType;
	newSetpoints.panValue = panValue;
	newSetpoints.tiltCtrltype = tiltType;
	newSetpoints.tiltValue = tiltValue;
}

void TATrackingMode::setNewPanSetpoint(TActrlType type, double _value) {
	newSetpoints.panCtrltype = type;
	newSetpoints.panValue = _value;
}
void TATrackingMode::setNewTiltSetpoint(TActrlType type, double _value) {
	newSetpoints.tiltCtrltype = type;
	newSetpoints.tiltValue = _value;
}

} /* namespace tracking */
