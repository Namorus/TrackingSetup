/*
 * TAConfig.h
 *
 *  Created on: Jun 15, 2013
 *      Author: thomas
 */

#ifndef TACONFIG_H_
#define TACONFIG_H_

#include "TAtypes.h"
#include "TAClass.h"

#include <map>

namespace tracking {

class TAConfig: public TAClass {
public:

	TAConfig();

	int readConfig(std::string filename);

	static int handler(void* user, const char* section, const char* name,
			const char* value);

	static bool tob(const char* value);

	static void display(TAConfig* pconfig);

	void display();

	void setNewConfig(TAConfig* pNewCfg);

	configChanges getConfigChanges();

	GlobalConf Glbl;
	GPStrackingConf GPS;
	MotorControlConf Mot;
	LocateConf locate;
	FindNorthConf findNorth;

	friend std::ostream& operator<<(std::ostream& out, const TAConfig& cfg);
	friend std::istream& operator>>(std::istream& in, TAConfig& cfg);

private:
	std::string configFile;

	configChanges changelist;

};

} /* namespace tracking */
#endif /* TACONFIG_H_ */
