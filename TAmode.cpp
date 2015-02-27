/*
 * TAmode.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: thomas
 */

#include "TAmode.h"

namespace tracking {

TAmode::TAmode() :
		curmode(tm_INIT) {
	pMutexCurmode = new pthread_mutex_t;
	pthread_mutex_init(pMutexCurmode, NULL);
}

trackingMode TAmode::get() {
	trackingMode temp;
	pthread_mutex_lock(pMutexCurmode);
	temp = curmode;
	pthread_mutex_unlock(pMutexCurmode);

	return temp;
}

void TAmode::set(trackingMode newmode) {
	pthread_mutex_lock(pMutexCurmode);
	curmode = newmode;
	pthread_mutex_unlock(pMutexCurmode);
	addLogMessage(vl_INFO, "Changing to mode " + getModeName(newmode));
}

std::string TAmode::getModeName(trackingMode mode) {
	switch (mode) {
	case tm_ENDING:
		return std::string("ENDING");
	case tm_INIT:
		return std::string("INIT");
	case tm_GPS_TRACKING:
		return std::string("GPS TRACKING");
	case tm_LOCATE:
		return std::string("LOCATE");
	case tm_MAPPING_ESTIMATION:
		return std::string("MAPPING ESTIMATION");
	case tm_READY:
		return std::string("READY");
	case tm_STOP:
		return std::string("STOP");
	}
	return std::string("unknown");
}

std::ostream& operator<<(std::ostream& out, const TAmode& curMode) {
	out << "$CurMode ";
	out << curMode.curmode;

	return out;
}

} /* namespace tracking */
