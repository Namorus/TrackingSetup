/*
 * TAClass.h
 *
 *  Created on: Jun 15, 2013
 *      Author: thomas
 */

#ifndef TACLASS_H_
#define TACLASS_H_

#include "TAtypes.h"

#include <list>

namespace tracking {

class TAClass {
public:
	std::list<TALogMessage>* getLog();
	void appendLog(std::list<TALogMessage>*);
	void addLogMessage(verbosityLevel, std::string message);

	void addLogChild(TAClass* child);
	void removeChild(TAClass* child);

protected:
	std::list<TALogMessage> logMessages;
	std::list<TAClass*> logChildren;

};

} /* namespace tracking */
#endif /* TACLASS_H_ */
