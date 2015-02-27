/*
 * TAConfig.h
 *
 *  Created on: Jun 15, 2013
 *      Author: thomas
 */

#ifndef TACONFIG_H_
#define TACONFIG_H_

#include <trackingsetup/trackingsetup.h>
#include <trackingsetup/types.h>
#include <map>

namespace tracking {

class TAConfig: public TrackingSetup {
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

	friend std::ostream& operator<<(std::ostream& out, const TAConfig& cfg);
	friend std::istream& operator>>(std::istream& in, TAConfig& cfg);

	GlobalConf Glbl;
	GPStrackingConf GPS;
	MotorControlConf Mot;
	LocateConf locate;
	FindNorthConf findNorth;



private:
	std::string configFile;
	configChanges changelist;

};

} /* namespace tracking */
#endif /* TACONFIG_H_ */
