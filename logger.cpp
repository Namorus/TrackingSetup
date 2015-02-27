/*
 * TALogger.cpp
 *
 *  Created on: Jun 15, 2013
 *      Author: thomas
 */

#include <trackingsetup/logger.h>
#include <trackingsetup/trackingsetup.h>
#include <list>

#include "iniparser/ini.h"

namespace tracking {

TALogger::TALogger(std::string logFileName, verbosityLevel _verbosity,
		bool _useStdout) :
		verbosity(_verbosity), useStdout(_useStdout), cacheLogForGui(false) {
	// init mutex
	pGuiBufferMutex = new pthread_mutex_t;
	pthread_mutex_init(pGuiBufferMutex, NULL);

	// open logfile to append log
	logFileStream.open(logFileName.c_str(),
			std::fstream::out | std::fstream::app);
	if (!logFileStream.is_open()) {
		log(vl_ERROR,
				"Could not open logfile (" + logFileName
						+ "). Check permissions.");
	}
}
void TALogger::log(verbosityLevel _vLvl, std::string _message) {
	time_t timestamp;
	time(&timestamp);
	log(timestamp, _vLvl, _message);
}
void TALogger::log(time_t timestamp, verbosityLevel _vLvl,
		std::string _message) {
	if ((int) _vLvl <= (int) verbosity) {
		std::string errormesage = "[" + verbosityLevelName(_vLvl) + "] "
				+ _message;

		if (useStdout) {
			std::cout << errormesage << std::endl;
		}

		struct tm * timeinfo;
		char timeBuffer[80];

		timeinfo = localtime(&timestamp);

		strftime(timeBuffer, 80, "%F %X - ", timeinfo);
		if (logFileStream.good()) {
			logFileStream << timeBuffer << errormesage << std::endl;
		}

		if (cacheLogForGui) {
			GuiStreambuffer << timeBuffer << errormesage << "#|#";
		}

	}

}

void TALogger::add(std::list<TALogMessage>* logList) {
	while (logList->size() > 0) {
//		std::cout << logList->pop_front() << std::endl;
		TALogMessage temp = logList->front();
		log(temp.timestamp, temp.vLvl, temp.message);
		logList->pop_front();
	}

}

std::string TALogger::verbosityLevelName(verbosityLevel verbosityLvl) {
	std::string name;
	switch (verbosityLvl) {
	case vl_ERROR:
		name = "ERROR";
		break;
	case vl_WARNING:
		name = "WARNING";
		break;
	case vl_INFO:
		name = "INFO";
		break;
	case vl_DEBUG:
		name = "DEBUG";
		break;
	}
	return name;
}

void TALogger::registerInstance(TrackingSetup* instance) {
	loggingInstances.push_back(instance);
}

void TALogger::fetchLogs() {
	for (std::list<TrackingSetup*>::iterator it = loggingInstances.begin();
			it != loggingInstances.end(); it++) {
		add((*it)->getLog());
	}
}

void TALogger::setCacheLogForGui(bool val) {
	cacheLogForGui = val;
}

void TALogger::clearCacheForGui() {
	pthread_mutex_lock(pGuiBufferMutex);
	GuiStreambuffer.str(std::string());
	pthread_mutex_unlock(pGuiBufferMutex);
}

std::ostream& operator<<(std::ostream& out, const TALogger& log) {
	out << "$Log ";
	pthread_mutex_lock(log.pGuiBufferMutex);
	out << log.GuiStreambuffer.str();
	pthread_mutex_unlock(log.pGuiBufferMutex);
	return out;
}

} /* namespace tracking */
