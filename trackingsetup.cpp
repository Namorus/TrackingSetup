//============================================================================
// Name        : TrackingSetup.cpp
// Author      : Thomas Mantel
// Version     : 1
// Copyright   : GPL
// Description : TrackingSetup
//============================================================================
#include <trackingsetup/find_north.h>
#include <trackingsetup/gui_backend.h>
#include <trackingsetup/mavlink_mag.h>
#include <trackingsetup/mavlink_radio_status.h>
#include <algorithm>
#include <list>
#include <sstream>
#include <unistd.h>

#include "trackingsetup/trackingsetup.h"

using namespace std;
using namespace tracking;

commandLineOptions parseCommandLine(int argc, char** argv);

namespace tracking {
std::list<TALogMessage>* TrackingSetup::getLog() {
//	std::list<TALogMessage> logCopy(logMessages);
//	logMessages.clear();
	if (!logChildren.empty()) {
		for (std::list<TrackingSetup*>::iterator it = logChildren.begin();
				it != logChildren.end(); it++) {
			appendLog((*it)->getLog());
		}
	}
	return &logMessages;
}

void TrackingSetup::appendLog(std::list<TALogMessage>* messages) {
	for (std::list<TALogMessage>::iterator it = messages->begin();
			it != messages->end(); it++) {
		logMessages.push_back(*it);
	}
	messages->clear();
}
void TrackingSetup::addLogMessage(verbosityLevel _verbosityLevel,
		std::string _message) {
	TALogMessage temp;
	time(&temp.timestamp);
	temp.vLvl = _verbosityLevel;
	temp.message = _message;

	logMessages.push_back(temp);
}

void TrackingSetup::addLogChild(TrackingSetup* child) {
	logChildren.push_back(child);
}

void TrackingSetup::removeChild(TrackingSetup* child) {
	std::list<TrackingSetup*>::iterator it = find(logChildren.begin(),logChildren.end(), child);
	logChildren.erase(it);
}

} /* namespace tracking */


int main(int argc, char** argv) {
	/* read command line input */
	struct commandLineOptions cmdLineOpts = parseCommandLine(argc, argv);

	/* enable logger */
	TALogger TAlog(cmdLineOpts.logFileName, cmdLineOpts.verbosity,
			cmdLineOpts.useStdout);

	/* log start */
	TAlog.log(vl_INFO, "TrackingSetup started");

	/* read config file */
	TAConfig TAcfg;
	TAcfg.readConfig(cmdLineOpts.cfgFileName);
	TAlog.add(TAcfg.getLog());
	if (cmdLineOpts.verbosity == vl_DEBUG) {
		TAcfg.display();
	}
	TAlog.registerInstance(&TAcfg);

	/* ***************** *
	 * initialize inputs *
	 * ***************** * */

	// MAVLink reader for local mavlink stream
	TSmavlinkReader localMavlinkReader(TAcfg.GPS.localMavlinkPort,
			TAcfg.GPS.localMavlinkBaudrate);
	TAlog.add(localMavlinkReader.getLog());
	TAlog.registerInstance(&localMavlinkReader);
    MavlinkMessages localMavlinkMessages;

	if (!cmdLineOpts.noLocalGPS) {
		if (localMavlinkReader.start() != true)
			cmdLineOpts.noLocalGPS = true; // reuse of variable
	}

	// local GPS
    TSmavlinkGPS localGps(&localMavlinkMessages, "localGPS");
	TAlog.add(localGps.getLog());
	TAlog.registerInstance(&localGps);

//	TSlocalGPS GPSSensor (&localMavlinkReader);

//	TAlog.add(GPSSensor.getLog());
//	TAlog.registerInstance(&GPSSensor);

	GPSPos localPosition;
    localPosition = TAcfg.GPS.AntennaPos;

	// MAVLink reader for remote mavlink stream
    TSmavlinkReader remoteMavlinkReader(TAcfg.GPS.remoteMavlinkPort,TAcfg.GPS.remoteMavlinkBaudrate);
	TAlog.add(remoteMavlinkReader.getLog());
	TAlog.registerInstance(&remoteMavlinkReader);
    MavlinkMessages remoteMavlinkMessages;
    TSmavlinkRadioStatus radioStatus(&remoteMavlinkMessages);
    RadioRSSI mavlinkRSSI;

	if (!cmdLineOpts.noRemoteGPS) {
		if (remoteMavlinkReader.start() != true)
			cmdLineOpts.noRemoteGPS = true; // reuse of variable
	}

	// remote GPS

    TSmavlinkGPS remoteGps(&remoteMavlinkMessages, "remoteGPS");
	TAlog.add(remoteGps.getLog());
	TAlog.registerInstance(&remoteGps);

	GPSPos remotePosition;

	/* initialize motor control */
	TAMotorControl motorControl;
	if (!cmdLineOpts.noMotors) {
//		TAlog.log(vl_DEBUG,"motorControl() successful");
		motorControl.init(&TAcfg.Mot);
//		TAlog.log(vl_DEBUG,"motorControl.init successful");
		TAlog.registerInstance(&motorControl);
//		TAlog.log(vl_DEBUG,"motorControl registered as a logging instance");
	}
	setpoints motorSetpoints;

	/* initialize tracking modes */
	TAmode curMode;
	TAlog.add(curMode.getLog());
	TAlog.registerInstance(&curMode);

	// GPS Tracking
	TAGPSTracking GPStracking;
	TAlog.add(GPStracking.getLog());
	TAlog.registerInstance(&GPStracking);

	// Magnetometer readings
    TSmavlinkMag mavlinkMagn(&localMavlinkMessages);
	TAlog.add(mavlinkMagn.getLog());
	TAlog.registerInstance(&mavlinkMagn);

	MagReading localMagn;

	// Find North
	TSfindNorth findNorth;
	TAlog.add(findNorth.getLog());
	TAlog.registerInstance(&findNorth);

	// Data Recorder
	TARecorder recorder;
	TAlog.add(recorder.getLog());
	TAlog.registerInstance(&recorder);

	/* set up GUI socket */
	TAGUIBackend GUIBackend;
	GUIBackend.setCfg(&TAcfg);
	GUIBackend.setClo(&cmdLineOpts);
	GUIBackend.setLog(&TAlog);
	GUIBackend.setLocalPos(&localPosition);
	GUIBackend.setRemotePos(&remotePosition);
	GUIBackend.setLocalGps(&localGps);
	GUIBackend.setRemoteGps(&remoteGps);
	GUIBackend.setCurMode(&curMode);
	GUIBackend.setGpStracking(&GPStracking);
	GUIBackend.setMotorControl(&motorControl);
	GUIBackend.setMotorSetpoints(&motorSetpoints);
	GUIBackend.setRecorder(&recorder);

	GUIBackend.startThread(6556);
	TAlog.add(GUIBackend.getLog());
	TAlog.registerInstance(&GUIBackend);

//	pthread_mutex_lock(GUIBackend.pControlMutex);

	// TODO: GUI & GUI socket

	/****************
	 * control loop *
	 ****************/
	// acquire measurements
	// process them
	// update antenna direction
	// send data to remote control GUI if necessary
	// read data from remote control GUI
	/* sanity check */
	// last chance to stop before control loop is started
	if (!motorControl.commOK()) {
		TAlog.log(vl_ERROR,
				"Quitting because of communication problems with EPOS");
		curMode.set(tm_ENDING);
	}

	/* declare (& initialize) some variables needed in control loop */
	timespec startTs, curTs;
	int waitDuration;
	int updatePeriod = 1e6 / TAcfg.Glbl.updateFreq;

	bool newTrackedPos = false;

	bool localGpsFixAcquired = false;

	float curPanAngle = 0, curTiltAngle = 0;

	if (cmdLineOpts.noLocalGPS) {
		localPosition = TAcfg.GPS.AntennaPos;
//		GPStracking.setAntennaPos(antennaPosition);
		TAlog.log(vl_INFO,
				"Using GPS position from configuration as antenna position");
	}

	std::vector<float> RSSvalues;
	std::vector<float> pktRates;

	TAlog.log(vl_INFO, "Starting control loop...");

	while (curMode.get() != tm_ENDING) {
		// get time
		clock_gettime(CLOCK_REALTIME, &startTs);
//        motorControl.displayDigitalInputState();
		/* get input and process it if necessary */
		// check if new config is available and process it
		configChanges cfgChanges = TAcfg.getConfigChanges();
		if (cfgChanges.anyChanges) {
			if (cfgChanges.Glbl) {
				updatePeriod = 1e6 / TAcfg.Glbl.updateFreq;
			}
			if (cfgChanges.GPS) {
				// TODO: changed GPS settings
			}
			if (cfgChanges.Mot) {
				// nothing to do at the moment, motor configuration may not be changed from GUI
			}
			if (cfgChanges.locate) {
				// nothing has to be done, configuration is passed on function call
			}
			GUIBackend.sendConfig();
		}

		// read out current angles
		if (!cmdLineOpts.noMotors) {
//			TAlog.log(vl_DEBUG,"Fetching motor data");
			motorControl.fetchData();
//			TAlog.log(vl_DEBUG,"Motor data successfully fetched");
			curPanAngle = motorControl.getPanAngle();
			curTiltAngle = motorControl.getTiltAngle();
		}
//        printf(".");
        // read MAVLink messages
        localMavlinkReader.getMavlinkMessages(localMavlinkMessages);
        remoteMavlinkReader.getMavlinkMessages(remoteMavlinkMessages);
        radioStatus.getRSSI(&mavlinkRSSI);
//        printf("RSSI: %d\n",mavlinkRSSI.rssi);
//        printf("\n");

        // process local GPS
		if(!cmdLineOpts.noLocalGPS) {
			localGps.getPos(&localPosition);
		}

		// process remote GPS
		newTrackedPos = remoteGps.getPos(&remotePosition);

		// check whether local GPS fix is acquired
		// once it is, calculate magnetic declination for current location
		if (!localGpsFixAcquired && localGps.getFixType() == 3) {
			localGpsFixAcquired = true;
			float magneticDeclination = findNorth.magneticDeclination(localPosition);
			cout << localPosition << endl;
			GPStracking.setMagneticDeclination(magneticDeclination);
			stringstream logmessage;
			logmessage << "Magnetic declination: " << magneticDeclination;
			TAlog.log(vl_DEBUG,logmessage.str());
		}

		// cout << "Remote Position: " << remotePosition.toString() << endl;

		/*
		 * call routine of current mode
		 */
		motorSetpoints.panCtrltype = ct_undefined;
		motorSetpoints.tiltCtrltype = ct_undefined;
		switch (curMode.get()) {
		case tm_ENDING:
			motorSetpoints.panCtrltype = ct_velocity;
			motorSetpoints.panValue = 0;
			motorSetpoints.tiltCtrltype = ct_velocity;
			motorSetpoints.tiltValue = 0;
			break;
		case tm_INIT:
			if (!cmdLineOpts.noMotors) {
				if (motorControl.isInitialized()) {
					if (!motorControl.tiltMotorIsHoming()) {
						motorControl.homeTiltMotor();
						TAlog.log(vl_INFO, std::string("Homing tilt motor"));
					} else if (motorControl.isTiltHomed()) {
						TAlog.log(vl_INFO, std::string("Tilt motor homed"));
                        curMode.set(tm_MAPPING_ESTIMATION);
					}
				}
			} else
				curMode.set(tm_MAPPING_ESTIMATION);
			break;

		case tm_GPS_TRACKING:
			if (newTrackedPos) {
				TAlog.log(vl_DEBUG,
						"processing new position of tracked object.");
				GPStracking.update(localPosition, remotePosition);
				motorSetpoints = GPStracking.getNewSetpoints();
			}
			break;

		case tm_MAPPING_ESTIMATION:
			// process magnetometer reading
			mavlinkMagn.getMag(&localMagn);

			if (findNorth.getLocateState() == fn_NOTREADY) {
				findNorth.init(TAcfg.findNorth.panSpeed,TAcfg.findNorth.tiltAngle);
			}
			else if (findNorth.getLocateState() == fn_FINISHED) {
				GPStracking.setMapping(findNorth.northPanAngleFound(),0);
				curMode.set(tm_GPS_TRACKING);
				findNorth.reset();
			}
			else {
                bool panPosReached = (!cmdLineOpts.noMotors) ? motorControl.panPositionReached() : true;
				bool tiltPosReached = (!cmdLineOpts.noMotors) ? motorControl.tiltPositionReached() : true;
				findNorth.update(curPanAngle,curTiltAngle,&localMagn,panPosReached,tiltPosReached);
			}
			motorSetpoints = findNorth.getNewSetpoints();

			break;
		case tm_READY:
			motorSetpoints = GUIBackend.getNewMotorSetpoints();
			break;

        case tm_LOCATE:
            break;

		case tm_STOP:
			motorSetpoints.panCtrltype = ct_velocity;
			motorSetpoints.panValue = 0;
			motorSetpoints.tiltCtrltype = ct_velocity;
			motorSetpoints.tiltValue = 0;
			if (!cmdLineOpts.noMotors) {
				if (motorControl.getTiltSpeed() == 0
						&& motorControl.getPanSpeed() == 0) {
					// Motors have come to a full stop, switch back to mode READY
					motorControl.enablePan();
					motorControl.enableTilt();
					curMode.set(tm_READY);
				}
			}
			break;
		}

		// set output (motor control)
		if (!cmdLineOpts.noMotors) {
			// pan
			switch (motorSetpoints.panCtrltype) {
			case ct_abspos:
				motorControl.setPanAngle(motorSetpoints.panValue);
				break;
			case ct_relpos:
				motorControl.setRelativePanAngle(motorSetpoints.panValue);
				break;
			case ct_velocity:
				motorControl.setPanVelocity(motorSetpoints.panValue);
				break;
			case ct_init:
				// if(motorSetpoints.panValue > 0) motorControl.initPanMotor()
				break;
			case ct_undefined:
				// do nothing
				break;
			}
			// tilt
			switch (motorSetpoints.tiltCtrltype) {
			case ct_abspos:
				motorControl.setTiltAngle(motorSetpoints.tiltValue);
				break;
			case ct_relpos:
				motorControl.setRelativeTiltAngle(motorSetpoints.tiltValue);
				break;
			case ct_velocity:
				motorControl.setTiltVelocity(motorSetpoints.tiltValue);
				break;
			case ct_init:
				if (motorSetpoints.tiltValue > 0)
					motorControl.homeTiltMotor();
				break;
			case ct_undefined:
				// do nothing
				break;
			}
		}
		// record
		recorder.record(&startTs, &RSSvalues, motorControl.getPanPosition(),
				motorControl.getTiltPosition(), &localPosition, &remotePosition,
				(int) curMode.get());

		// get log messages
		TAlog.fetchLogs();

		// signal to backend to receive and send data
		pthread_cond_signal(GUIBackend.pDataReadyCond);

		clock_gettime(CLOCK_REALTIME, &curTs);
		waitDuration = (int) (1e6 * (curTs.tv_sec - startTs.tv_sec)
				+ (curTs.tv_nsec - startTs.tv_nsec) / 1000);
		waitDuration = updatePeriod - waitDuration;
		if (waitDuration > 0) {
			usleep(waitDuration);
		}

	}
	/* end tracking antenna */
	GUIBackend.killThread();
//	remoteGPS.closeGPSPosServer();
//	GPSSensor.killThread();
	localMavlinkReader.stopReading();
	remoteMavlinkReader.stopReading();
	if (!cmdLineOpts.noMotors) {
		motorControl.stop();
	}
	TAlog.fetchLogs();
	TAlog.log(vl_INFO, "Tracking Antenna closed");
	return EXIT_SUCCESS;
}

commandLineOptions parseCommandLine(int argc, char** argv) {
	commandLineOptions cLO;

	for (int i = 1; i < argc; i++) {
		if (argv[i][0] == '-') {
			std::string setting(argv[i]);
			int valuepos = setting.find("=");
			std::string option = setting.substr(1, valuepos - 1);
			std::string value = setting.substr(valuepos + 1, setting.size());

			if (option.compare("cfgFile") == 0) {
				cLO.cfgFileName = value;
			} else if (option.compare("logFile") == 0) {
				cLO.logFileName = value;
			} else if (option.compare("v") == 0) {
				cLO.verbosity = (verbosityLevel) atoi(value.c_str());
			} else if (option.compare("noMotors") == 0) {
				cLO.noMotors = true;
			} else if (option.compare("noLocalGPS") == 0) {
				cLO.noLocalGPS = true;
			} else if (option.compare("noRemoteGPS") == 0) {
				cLO.noRemoteGPS = true;
			} else if (option.compare("noWLAN") == 0) {
				cLO.noWLAN = true;
			} else if (option.compare("useTUI") == 0) {
				cLO.useTUI = true;
			} else if (option.compare("useStdout") == 0) {
				cLO.useStdout = true;
			}
		}
	}
	if (cLO.verbosity == vl_DEBUG) {
		cout << "cfgFileName \t= \t" << cLO.cfgFileName << endl;
		cout << "logFileName \t= \t" << cLO.logFileName << endl;
		cout << "vebosityLevel \t= \t" << cLO.verbosity << endl;
		if (cLO.noMotors)
			cout << "not using motors" << endl;
		if (cLO.noLocalGPS)
			cout << "not using local GPS sensor" << endl;
		if (cLO.noRemoteGPS)
			cout << "not using remote GPS sensor" << endl;
		if (cLO.noWLAN)
			cout << "not using WLAN" << endl;
		if (cLO.useTUI)
			cout << "using text user interfaces (based on ncurses)" << endl;
		if (cLO.useStdout)
			cout << "using stdout for log/error messages" << endl;
	}
	return cLO;
}
