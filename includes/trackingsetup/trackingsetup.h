/*
 * TAClass.h
 *
 *  Created on: Jun 15, 2013
 *  Last change on: Feb 27, 2015
 *      Author: Thomas Mantel
 */

#ifndef TACLASS_H_
#define TACLASS_H_

#include <trackingsetup/types.h>
#include <list>



namespace tracking {

class TrackingSetup {
public:

	/** \brief read out the buffer of log messages
	 *
	 * Appends the log messages of its children (if any) to its own buffer before returning the pointer to the buffer.
	 *
	 * @return A pointer to a list of log messages
	 */
	std::list<TALogMessage>* getLog();

	/** \brief Append log to log of calling instance
	 *
	 * All the log messages of the list supplied are added to the log message buffer of the calling instance.
	 *
	 * @param logMessageList Pointer to list of log messages that should be appended
	 */
	void appendLog(std::list<TALogMessage>* logMessageList);

	/** \brief Add a log message to log buffer
	 *
	 * @param verbosity_level Defines the verbosity level of the message
	 * @param message String containing the log message
	 */
	void addLogMessage(verbosityLevel verbosity_level, std::string message);

	/** \brief Add an instance of class TrackingSetup as a child
	 *
	 * When reading out log messages with getLog(), all log messages of children added
	 * with this function will also be returned.
	 *
	 * Be carefull with this function, there is no check for loops!
	 *
	 * @param child Pointer to child that should be added.
	 */
	void addLogChild(TrackingSetup* child);

	/** \brief Remove a previously added instance of class TrackingSetup from the list of children.
	 *
	 * @param child Pointer of child that should be removed.
	 */
	void removeChild(TrackingSetup* child);


protected:
	std::list<TALogMessage> logMessages;
	std::list<TrackingSetup*> logChildren;

};

} /* namespace tracking */
#endif /* TACLASS_H_ */
