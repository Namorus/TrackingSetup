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

class Config: public TrackingSetup {
public:

	Config();

	int readConfig(std::string filename);

	static int handler(void* user, const char* section, const char* name,
			const char* value);

	static bool tob(const char* value);

	static void display(Config* pconfig);

	void display();

	void setNewConfig(Config* pNewCfg);

	configChanges getConfigChanges();

	friend std::ostream& operator<<(std::ostream& out, const Config& cfg);
	friend std::istream& operator>>(std::istream& in, Config& cfg);

	GlobalConf Glbl;
	GPStrackingConf GPS;
	MotorControlConf Mot;
	LocateConf locate;
	FindNorthConf findNorth;
	EstimatorConf Estimator;


private:
	std::string configFile;
	configChanges changelist;

};

} /* namespace tracking */
#endif /* TACONFIG_H_ */
