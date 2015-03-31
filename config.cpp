/*
 * TAConfig.cpp
 *
 *  Created on: Jun 15, 2013
 *      Author: thomas
 */

#include <trackingsetup/config.h>
#include "includes/iniparser/ini.h"

namespace tracking {

Config::Config() :
	configFile("") {

}

int Config::readConfig(std::string filename) {
	configFile = filename;
	if (configFile.size() > 0) {
		if (ini_parse(configFile.c_str(), handler, (void*) this) < 0) {
			this->addLogMessage(vl_ERROR, "Can't load \"" + configFile + "\"");
			return 1;
		}
		this->addLogMessage(vl_INFO,
				"Configuration successfully read from \"" + configFile + "\"");

		return 0;
	}
	this->addLogMessage(vl_ERROR, "No config file has been specified");

	return 1;
}

int Config::handler(void* user, const char* section, const char* name,
		const char* value) {
	Config* pconfig = (Config*) user;

#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0

	// global
//    if (MATCH("global", "use_ncurses")) {
//    	pconfig->Glbl.use_ncurses = tob(value);
//    } else
	if (MATCH("global", "verbose")) {
		pconfig->Glbl.verbose = atoi(value);
	} else if (MATCH("global", "CtrlUpdateFreq")) {
		pconfig->Glbl.updateFreq = atoi(value);
//    } else if (MATCH("global", "GuiUpdateFreq")) {
//    	pconfig->Glbl.GuiUpdateFreq = atoi(value);
	}

	// GPS
	else if (MATCH("GPS", "Antenna.lat")) {
		pconfig->GPS.AntennaPos.lat = atof(value);
	} else if (MATCH("GPS", "Antenna.lon")) {
		pconfig->GPS.AntennaPos.lon = atof(value);
	} else if (MATCH("GPS", "Antenna.elev")) {
		pconfig->GPS.AntennaPos.alt = atof(value);
	} else if (MATCH("GPS", "localMavlinkVid")) {
		pconfig->GPS.localMavlinkVid = strtol(value,NULL,0);
	} else if (MATCH("GPS", "localMavlinkPid")) {
		pconfig->GPS.localMavlinkPid = strtol(value,NULL,0);
	} else if (MATCH("GPS", "localMavlinkInterface")) {
		pconfig->GPS.localMavlinkInterface = atoi(value);
	} else if (MATCH("GPS", "localMavlinkBaudrate")) {
		pconfig->GPS.localMavlinkBaudrate = atoi(value);
	} else if (MATCH("GPS", "remoteMavlinkVid")) {
		pconfig->GPS.remoteMavlinkVid = strtol(value,NULL,0);
	} else if (MATCH("GPS", "remoteMavlinkPid")) {
		pconfig->GPS.remoteMavlinkPid = strtol(value,NULL,0);
	} else if (MATCH("GPS", "remoteMavlinkInterface")) {
		pconfig->GPS.remoteMavlinkInterface = atoi(value);
	} else if (MATCH("GPS", "remoteMavlinkBaudrate")) {
		pconfig->GPS.remoteMavlinkBaudrate = atoi(value);
	}

	// MotorControl
	else if (MATCH("MotorControl", "commRetries")) {
		pconfig->Mot.commRetries = atoi(value);
	} else if (MATCH("MotorControl", "panEposNo")) {
		pconfig->Mot.panEposNo = value;
	} else if (MATCH("MotorControl", "tiltEposNo")) {
		pconfig->Mot.tiltEposNo = value;
	}

	// Locate
	else if (MATCH("RSSLocate", "locatePanSpeed")) {
		pconfig->locate.panSpeed = atoi(value);
	} else if (MATCH("RSSLocate", "tiltAngleStart")) {
		pconfig->locate.tiltAngleStart = atoi(value);
	} else if (MATCH("RSSLocate", "tiltAngleStop")) {
		pconfig->locate.tiltAngleStop = atoi(value);
	} else if (MATCH("RSSLocate", "pitch")) {
		pconfig->locate.pitch = atoi(value);
	}

    // FindNorth
    else if (MATCH("FindNorth", "panSpeed")) {
        pconfig->findNorth.panSpeed = atoi(value);
    } else if (MATCH("FindNorth", "tiltAngle")) {
        pconfig->findNorth.tiltAngle = atoi(value);
    }

	else {
		return 0; /* unknown section/name, error */
	}

	return 1;
}

bool Config::tob(const char* value) {
	return (strcmp(value, "1") == 0 || strcmp(value, "true") == 0
			|| strcmp(value, "yes") == 0);
}

void Config::display(Config* pconfig) {
	using namespace std;

	cout << "TrackingAntenna Configuration" << endl
			<< "*****************************" << endl << endl
			<< "[Global config]" << endl << "verbose: \t\t\t"
			<< pconfig->Glbl.verbose << endl
			<< "update frequency of ctrl loop: " << pconfig->Glbl.updateFreq
			<< endl << endl;
//			<< "using ncurses: \t\t\t" << pconfig->Glbl.use_ncurses << endl << endl;

	cout << "[GPStracking config]" << endl << "Antenna Position: \t\t"
			<< pconfig->GPS.AntennaPos.lat << "N, "
			<< pconfig->GPS.AntennaPos.lon << "E @ "
			<< pconfig->GPS.AntennaPos.alt << "m " << endl
			<< "PX4 Mavlink device: \t\tvid=0x" << std::hex << pconfig->GPS.localMavlinkVid << ", pid=0x" << pconfig->GPS.localMavlinkPid << ", interface=" << std::dec << pconfig->GPS.localMavlinkInterface << "" << endl
			<< "PX4 Mavlink baudrate: \t\t" << pconfig->GPS.localMavlinkBaudrate << endl
			<< "3DR radio Mavlink device: \tvid=0x" << std::hex << pconfig->GPS.remoteMavlinkVid << ", pid=0x" << pconfig->GPS.remoteMavlinkPid << ", interface=" << std::dec << pconfig->GPS.remoteMavlinkInterface << "" << endl
			<< "3DR radio Mavlink baudrate: \t"	<< pconfig->GPS.remoteMavlinkBaudrate << endl
			<< endl;

//	cout << "[RSSlocate config]" << endl << "Pan speed: \t\t\t"
//			<< pconfig->locate.panSpeed << endl << "Starting tilt angle: \t\t"
//			<< pconfig->locate.tiltAngleStart << endl
//			<< "Stopping tilt angle: \t\t" << pconfig->locate.tiltAngleStop
//			<< endl << "Pitch: \t\t\t\t" << pconfig->locate.pitch << endl
//			<< endl;

    cout << "[FindNorth config]" << endl
         << "Pan speed: \t\t\t"	<< pconfig->findNorth.panSpeed << endl
         << "Starting tilt angle: \t\t"	<< pconfig->findNorth.tiltAngle << endl
         << endl;

	cout << "[Motor control config]" << endl << "Serial no pan Epos: \t\t"
			<< pconfig->Mot.panEposNo << endl << "Serial no tilt Epos: \t\t"
			<< pconfig->Mot.tiltEposNo << endl
			<< "Max retries for 1 command: \t" << pconfig->Mot.commRetries
			<< endl << endl << endl;
}

void Config::display() {
	display(this);
}

std::ostream& operator<<(std::ostream& out, const Config& cfg) {
	out << "$Config ";

	/* GlobalConf */
	out << cfg.Glbl.verbose << " " << cfg.Glbl.updateFreq << " ";

	/* GPStrackingConf */
	out << cfg.GPS.AntennaPos << " "
		<< cfg.GPS.localMavlinkVid << " "
		<< cfg.GPS.localMavlinkPid << " "
		<< cfg.GPS.localMavlinkInterface << " "
		<< cfg.GPS.localMavlinkBaudrate << " "
		<< cfg.GPS.remoteMavlinkVid << " "
		<< cfg.GPS.remoteMavlinkPid << " "
		<< cfg.GPS.remoteMavlinkInterface << " "
		<< cfg.GPS.remoteMavlinkBaudrate << " ";

	/* MotorControlConf */
	out << cfg.Mot.panEposNo << " " << cfg.Mot.tiltEposNo << " "
			<< cfg.Mot.commRetries << " ";

	/* LocateConf */
	out << cfg.locate.panSpeed << " " << cfg.locate.tiltAngleStart << " "
			<< cfg.locate.tiltAngleStop << " " << cfg.locate.pitch;

	return out;

}

void Config::setNewConfig(Config* pNewCfg) {

	//TODO: go through new config and check what has changed to signal the new changes to the classes
	if (pNewCfg->Glbl != Glbl) {
		changelist.Glbl = true;
		changelist.anyChanges = true;
		Glbl = pNewCfg->Glbl;
	}

	if (pNewCfg->GPS != GPS) {
		changelist.GPS = true;
		changelist.anyChanges = true;
		GPS = pNewCfg->GPS;
	}
	if (pNewCfg->Mot != Mot) {
		changelist.Mot = true;
		changelist.anyChanges = true;
		Mot = pNewCfg->Mot;
	}

	if (pNewCfg->locate != locate) {
		changelist.locate = true;
		changelist.anyChanges = true;
		locate = pNewCfg->locate;
	}
}

std::istream& operator>>(std::istream& in, Config& cfg) {

	/* GlobalConf */
	in >> cfg.Glbl.verbose;
	in >> cfg.Glbl.updateFreq;

	/* GPStrackingConf */
	in >> cfg.GPS.AntennaPos.lat;
	in >> cfg.GPS.AntennaPos.lon;
	in >> cfg.GPS.AntennaPos.alt;
	in >> cfg.GPS.localMavlinkVid;
	in >> cfg.GPS.localMavlinkPid;
	in >> cfg.GPS.localMavlinkInterface;
	in >> cfg.GPS.localMavlinkBaudrate;
	in >> cfg.GPS.remoteMavlinkVid;
	in >> cfg.GPS.remoteMavlinkPid;
	in >> cfg.GPS.remoteMavlinkInterface;
	in >> cfg.GPS.remoteMavlinkBaudrate;

	/* MotorControlConf */
	in >> cfg.Mot.panEposNo;
	in >> cfg.Mot.tiltEposNo;
	in >> cfg.Mot.commRetries;

	/* LocateConf */
	in >> cfg.locate.panSpeed;
	in >> cfg.locate.tiltAngleStart;
	in >> cfg.locate.tiltAngleStop;
	in >> cfg.locate.pitch;

	return in;
}

bool operator==(const GlobalConf& a, const GlobalConf& b) {
	if (a.verbose != b.verbose)
		return false;

	if (a.updateFreq != b.updateFreq)
		return false;

//	if(a.TrafficGen != b.TrafficGen)
//		return false;
	return true;
}
bool operator!=(const GlobalConf& a, const GlobalConf& b) {
	if (a == b)
		return false;

	return true;
}

bool operator==(const GPStrackingConf& a, const GPStrackingConf& b) {
	if (a.AntennaPos != b.AntennaPos)
		return false;

	if (a.localMavlinkVid != b.localMavlinkVid)
		return false;

	if (a.localMavlinkPid != b.localMavlinkPid)
		return false;

	if (a.localMavlinkInterface != b.localMavlinkInterface)
		return false;

	if (a.localMavlinkBaudrate != b.localMavlinkBaudrate)
		return false;

	if (a.remoteMavlinkVid != b.remoteMavlinkVid)
		return false;

	if (a.remoteMavlinkPid != b.remoteMavlinkPid)
		return false;

	if (a.remoteMavlinkInterface != b.remoteMavlinkInterface)
		return false;

	if (a.remoteMavlinkBaudrate != b.remoteMavlinkBaudrate)
		return false;



	return true;
}
bool operator!=(const GPStrackingConf& a, const GPStrackingConf& b) {
	if (a == b)
		return false;

	return true;
}

bool operator==(const MotorControlConf& a, const MotorControlConf& b) {
	if (a.panEposNo != b.panEposNo)
		return false;

	if (a.tiltEposNo != b.tiltEposNo)
		return false;

	if (a.commRetries != b.commRetries)
		return false;

	return true;
}
bool operator!=(const MotorControlConf& a, const MotorControlConf& b) {
	if (a == b)
		return false;

	return true;
}

bool operator==(const LocateConf& a, const LocateConf& b) {
	if (a.panSpeed != b.panSpeed)
		return false;

	if (a.tiltAngleStart != b.tiltAngleStart)
		return false;

	if (a.tiltAngleStop != b.tiltAngleStop)
		return false;

	if (a.pitch != b.pitch)
		return false;

	return true;
}
bool operator!=(const LocateConf& a, const LocateConf& b) {
	if (a == b)
		return false;

	return true;
}

configChanges Config::getConfigChanges() {
	configChanges temp = changelist;
	changelist = configChanges();
	return temp;
}

} /* namespace tracking */
