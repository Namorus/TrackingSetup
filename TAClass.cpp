/*
 * TAClass.cpp
 *
 *  Created on: Jun 15, 2013
 *      Author: thomas
 */

//#include "TAClass.h"
#include "TSincludes.h"

namespace tracking {
std::list<TALogMessage>* TAClass::getLog() {
//	std::list<TALogMessage> logCopy(logMessages);
//	logMessages.clear();
	if (!logChildren.empty()) {
		for (std::list<TAClass*>::iterator it = logChildren.begin();
				it != logChildren.end(); it++) {
			appendLog((*it)->getLog());
		}
	}
	return &logMessages;
}

void TAClass::appendLog(std::list<TALogMessage>* messages) {
	for (std::list<TALogMessage>::iterator it = messages->begin();
			it != messages->end(); it++) {
		logMessages.push_back(*it);
	}
	messages->clear();
}
void TAClass::addLogMessage(verbosityLevel _verbosityLevel,
		std::string _message) {
	TALogMessage temp;
	time(&temp.timestamp);
	temp.vLvl = _verbosityLevel;
	temp.message = _message;

	logMessages.push_back(temp);
}

void TAClass::addLogChild(TAClass* child) {
	logChildren.push_back(child);
}

void TAClass::removeChild(TAClass* child) {
	std::list<TAClass*>::iterator it = find(logChildren.begin(),
			logChildren.end(), child);
	logChildren.erase(it);
}

} /* namespace tracking */
