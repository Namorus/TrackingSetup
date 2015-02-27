/*
 * TAmode.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: thomas
 */

#include <trackingsetup/tracking_state.h>

namespace tracking {

State::State() :
		curmode(ts_INIT) {
	pMutexCurmode = new pthread_mutex_t;
	pthread_mutex_init(pMutexCurmode, NULL);
}

trackingSetupState State::get() {
	trackingSetupState temp;
	pthread_mutex_lock(pMutexCurmode);
	temp = curmode;
	pthread_mutex_unlock(pMutexCurmode);

	return temp;
}

void State::set(trackingSetupState newmode) {
	pthread_mutex_lock(pMutexCurmode);
	curmode = newmode;
	pthread_mutex_unlock(pMutexCurmode);
	addLogMessage(vl_INFO, "Changing to mode " + getModeName(newmode));
}

std::string State::getModeName(trackingSetupState mode) {
	switch (mode) {
	case ts_ENDING:
		return std::string("ENDING");
	case ts_INIT:
		return std::string("INIT");
	case ts_GPS_TRACKING:
		return std::string("GPS TRACKING");
	case ts_LOCATE:
		return std::string("LOCATE");
	case ts_FIND_NORTH:
		return std::string("MAPPING ESTIMATION");
	case ts_READY:
		return std::string("READY");
	case ts_STOP:
		return std::string("STOP");
	}
	return std::string("unknown");
}

std::ostream& operator<<(std::ostream& out, const State& curMode) {
	out << "$CurMode ";
	out << curMode.curmode;

	return out;
}

} /* namespace tracking */
