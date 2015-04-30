//============================================================================
// Name        : TrackingSetup.cpp
// Author      : Thomas Mantel
// Version     : 1
// Copyright   : GPL
// Description : TrackingSetup
//============================================================================
#include <algorithm>
#include <list>
#include <sstream>
#include <unistd.h>
#include <armadillo> // ROMAN	 matrix calculation library
//#include <GeographicLib/Geocentric.hpp>
//#include <GeographicLib/LocalCartesian.hpp>


#include <trackingsetup/trackingsetup.h>
#include <trackingsetup/find_north.h>
#include <trackingsetup/forward_calc.h>
#include <trackingsetup/gps_tracking.h>
#include <trackingsetup/gui_backend.h>
#include <trackingsetup/mavlink_mag.h>
#include <trackingsetup/mavlink_radio_status.h>
#include <trackingsetup/mavlink_gpos.h>

using namespace std;
using namespace tracking;
using namespace arma;// -------------------------------------------------ROMAN

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
	struct commandLineOptions commandLineOptions = parseCommandLine(argc, argv);

	/* enable logger */
	Logger trackingLog(commandLineOptions.logFileName, commandLineOptions.verbosity,
			commandLineOptions.useStdout);

	/* log start */
	trackingLog.log(vl_INFO, "TrackingSetup started");

	/* read config file */
	Config trackingConfig;
	trackingConfig.readConfig(commandLineOptions.cfgFileName);
	trackingLog.add(trackingConfig.getLog());
	if (commandLineOptions.verbosity == vl_DEBUG) {
		trackingConfig.display();
	}
	trackingLog.registerInstance(&trackingConfig);

	/* ***************** *
	 * initialize inputs *
	 * ***************** * */

	// MAVLink reader for local MAVLink stream
	MavlinkReader localMavlinkReader(trackingConfig.GPS.localMavlinkVid,trackingConfig.GPS.localMavlinkPid,trackingConfig.GPS.localMavlinkInterface,trackingConfig.GPS.localMavlinkBaudrate,"PX4");
	trackingLog.add(localMavlinkReader.getLog());
	trackingLog.registerInstance(&localMavlinkReader);
    MavlinkMessages localMavlinkMessages;

	if (!commandLineOptions.noLocalGPS) {
		if (localMavlinkReader.start() != true)
			commandLineOptions.noLocalGPS = true; // reuse of variable
	}

	// local GPS
    MavlinkGps localGps(&localMavlinkMessages, "localGPS");
	trackingLog.add(localGps.getLog());
	trackingLog.registerInstance(&localGps);

	GPSPos localPosition;
    localPosition = trackingConfig.GPS.AntennaPos;


    // set


	// MAVLink reader for remote MAVLink stream
    MavlinkReader remoteMavlinkReader(trackingConfig.GPS.remoteMavlinkVid,trackingConfig.GPS.remoteMavlinkPid,trackingConfig.GPS.remoteMavlinkInterface,trackingConfig.GPS.remoteMavlinkBaudrate,"3DR radio");
    trackingLog.add(remoteMavlinkReader.getLog());
	trackingLog.registerInstance(&remoteMavlinkReader);
    MavlinkMessages remoteMavlinkMessages;
    MavlinkRadioStatus radioStatus(&remoteMavlinkMessages);
    RadioRSSI mavlinkRSSI;

	if (!commandLineOptions.noRemoteGPS) {
		if (remoteMavlinkReader.start() != true)
			commandLineOptions.noRemoteGPS = true; // reuse of variable
	}

	// remote GPS
    MavlinkGps remoteGps(&remoteMavlinkMessages, "remoteGPS");
	trackingLog.add(remoteGps.getLog());
	trackingLog.registerInstance(&remoteGps);

	GPSPos remotePosition;

	// remote Global Position
	MavlinkGpos remoteGpos(&remoteMavlinkMessages);
	trackingLog.add(remoteGpos.getLog());
	trackingLog.registerInstance(&remoteGpos);

	// GlobalPos struct holding global position
	GlobalPos remoteGlobalPosition;

	// Estimator holding estimate of tracked object in local coordinates
	ForwardCalc remotePosEstimator;
	trackingLog.add(remotePosEstimator.getLog());
	trackingLog.registerInstance(&remotePosEstimator);

//	LocalPos estimatedRemotePosition;
//	LocalPos estimatedRemoteVelocity;

	// initialize estimator parameter // -------------------------------------------------ROMAN
	double kf_time_old = 0;
	double phi_old=0;
	double v_air_old=0;

	/* initialize motor control */
	MotorControl motorControl;
	if (!commandLineOptions.noMotors) {
		motorControl.init(&trackingConfig.Mot);
		trackingLog.registerInstance(&motorControl);
	}
	setpoints motorSetpoints;

	/* initialize tracking modes */
	State currentState;
	trackingLog.add(currentState.getLog());
	trackingLog.registerInstance(&currentState);

	// GPS Tracking
	GpsTrackingMode gpsTracking(&remotePosEstimator);
	trackingLog.add(gpsTracking.getLog());
	trackingLog.registerInstance(&gpsTracking);

	// Magnetometer readings
    MavlinkMagnetometer mavlinkMagn(&localMavlinkMessages);
	trackingLog.add(mavlinkMagn.getLog());
	trackingLog.registerInstance(&mavlinkMagn);

	MagReading localMagn;

	// Find North
	FindNorth findNorth;
	trackingLog.add(findNorth.getLog());
	trackingLog.registerInstance(&findNorth);

	// Data Recorder
	Recorder recorder;
	trackingLog.add(recorder.getLog());
	trackingLog.registerInstance(&recorder);

	/* set up GUI socket */
	GuiBackend guiBackend;
	guiBackend.setConfig(&trackingConfig);
	guiBackend.setCommandLineOptions(&commandLineOptions);
	guiBackend.setLog(&trackingLog);
	guiBackend.setLocalPos(&localPosition);
	guiBackend.setRemotePos(&remotePosition);
	guiBackend.setLocalGps(&localGps);
	guiBackend.setRemoteGps(&remoteGps);
	guiBackend.setCurMode(&currentState);
	guiBackend.setGpsTracking(&gpsTracking);
	guiBackend.setMotorControl(&motorControl);
	guiBackend.setMotorSetpoints(&motorSetpoints);
	guiBackend.setRecorder(&recorder);

	guiBackend.startThread(6556);
	trackingLog.add(guiBackend.getLog());
	trackingLog.registerInstance(&guiBackend);

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
		trackingLog.log(vl_ERROR,
				"Quitting because of communication problems with EPOS");
		currentState.set(ts_ENDING);
	}

	/* declare (& initialize) some variables needed in control loop */
	timespec startTs, curTs;
	int waitDuration;
	int updatePeriod = 1e6 / trackingConfig.Glbl.updateFreq;

	bool newAntennaPos = false;
	bool newTrackedPos = false;
	bool estimateUpdated = false;

	bool newMavlinkMessage = false; // -------------------------------------------------ROMAN

	bool localGpsFixAcquired = false;

	float curPanAngle = 0, curTiltAngle = 0;

	// Handle local GPS in case no receiver is used
	if (commandLineOptions.noLocalGPS) {
		localPosition = trackingConfig.GPS.AntennaPos;
		// localGpsFixAcquired = true;
		// set local position to instances where needed
		remotePosEstimator.setAntennaPos(localPosition);
		trackingLog.log(vl_INFO,"Using GPS position from configuration as antenna position");
	}

	std::vector<float> RSSvalues;
	std::vector<float> pktRates;

	trackingLog.log(vl_INFO, "Starting control loop...");

	while (currentState.get() != ts_ENDING) {
		// get time
		clock_gettime(CLOCK_REALTIME, &startTs);
//        motorControl.displayDigitalInputState();
		/* get input and process it if necessary */
		// check if new config is available and process it
		configChanges cfgChanges = trackingConfig.getConfigChanges();
		if (cfgChanges.anyChanges) {
			if (cfgChanges.Glbl) {
				updatePeriod = 1e6 / trackingConfig.Glbl.updateFreq;
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
			guiBackend.sendConfig();
		}

		// read out current angles
		if (!commandLineOptions.noMotors) {
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

        // add phi measurement and airspeed measurement // -------------------------------------------------ROMAN

        // newPhi =         ?????????????????????????????
        double phi;
        // AirSpeed =       ?????????????????????????????
        double v_air;

        newMavlinkMessage = true;
        //mavlink_message.timestamp=?????????????? // -------------------------------------------------ROMAN


        // process local GPS
		if(!commandLineOptions.noLocalGPS) {
			newAntennaPos = localGps.getPos(&localPosition);
		}

		// process remote GPS
		remoteGps.getPos(&remotePosition);

		// process remote GPOS
		newTrackedPos = remoteGpos.getGpos(&remoteGlobalPosition);

		// check whether local GPS fix is acquired
		// once it is, calculate magnetic declination for current location
		if (!localGpsFixAcquired && (localGps.getFixType() == 3 || commandLineOptions.noLocalGPS)) {
			localGpsFixAcquired = true;
			float magneticDeclination = findNorth.magneticDeclination(localPosition);
			gpsTracking.setMagneticDeclination(magneticDeclination);
			stringstream logmessage;
			logmessage << "Magnetic declination: " << magneticDeclination;
			trackingLog.log(vl_INFO,logmessage.str());
		}

		// cout << "Remote Position: " << remotePosition.toString() << endl;


// -------------------------------------------------ROMAN-Start---------------------------------------------------//

double kf_time = (startTs.tv_sec*1e6 + startTs.tv_nsec*1e-3); //???????????????????? // current KF start time


// define measurement vector
// consists of new GPS position/velocity measurement
arma::vec Z(6);
Z << remotePosEstimator.targetPosLocal_.y << endr
	 << remotePosEstimator.targetGlobalPos_.velocity.lat << endr
	 << remotePosEstimator.targetPosLocal_.x  << endr
	 << remotePosEstimator.targetGlobalPos_.velocity.lon << endr
	 << remotePosEstimator.targetPosLocal_.z << endr
	 << remotePosEstimator.targetGlobalPos_.velocity.alt << endr;

//--------------------------------------------------------------------------
// check if position measurement is available
//--------------------------------------------------------------------------
if (newTrackedPos){	//if p_pdot_time>=kf_time_old && p_pdot_time<kf_time

	//--------------------------------------------------------------------------
	// check if phi measurement is available
	//--------------------------------------------------------------------------
	if (newMavlinkMessage){ //phi_time>=kf_time_old && phi_time<kf_time %booth are available

		  // (re-)order timestamps
				        if (remoteGlobalPosition.localTimestamp > mavlink_message.timestamp) { // check if phi measurement (mavlink message) was first
				           double timestamp_1 = mavlink_message.timestamp;
				           double timestamp_2 = remoteGlobalPosition.localTimestamp;

				                double dt1 = timestamp_1 - kf_time_old; // calculate dt

				            // predict with phi_old
				            // [Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);
				                remotePosEstimator.predictEstimate(phi_old, v_air_old, dt1);

				                double dt2 = timestamp_2 - timestamp_1; // calculate dt

				            // predict with phi_new
				                // [Xhat2,~,P2]  = KF_predict(Xhat1,v_air,phi,P1,dt2,q,var_phi,gravity);
				                remotePosEstimator.predictEstimate(phi, v_air, dt2);

				            // udpate position
				                // [Xhat3,P3]  = KF_update_pos(Xhat2,p_pdot',P2,r_a,r_b);
				                remotePosEstimator.updateEstimate(Z);	//execution of update function

				                double dt3 = kf_time - remoteGlobalPosition.localTimestamp; // calculate dt

				            // predict with new position and new phi
				                // [Xhat_out,omega_out,P]  = KF_predict(Xhat3,v_air,phi,P3,dt3,q,var_phi,gravity);
				                remotePosEstimator.predictEstimate(phi, v_air, dt3);

				        }else if (remoteGlobalPosition.localTimestamp<mavlink_message.timestamp) { // check if GPOS was first
				            double timestamp_1 = remoteGlobalPosition.localTimestamp;
				            double timestamp_2 = mavlink_message.timestamp;

				                double dt1 = timestamp_1 - kf_time_old; // calculate dt

				            // predict
				                // [Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);
				                remotePosEstimator.predictEstimate(phi_old, v_air_old, dt1);

				            // udpate position
				                // [Xhat2,P2]  = KF_update_pos(Xhat1,p_pdot',P1,r_a,r_b);
				                remotePosEstimator.updateEstimate(Z);	//execution of update function

				                double dt2 = timestamp_2 - timestamp_1; // calculate dt

				            // predict with new position and old phi
				                // [Xhat3,~,P3]  = KF_predict(Xhat2,v_air_old,phi_old,P2,dt2,q,var_phi,gravity);
				                remotePosEstimator.predictEstimate(phi_old, v_air_old, dt2);

				                double dt3 = kf_time - remoteGlobalPosition.localTimestamp; // calculate dt

				            // predict with new position and new phi
				                // [Xhat_out,omega_out,P]  = KF_predict(Xhat3,v_air,phi,P3,dt3,q,var_phi,gravity);
				                remotePosEstimator.predictEstimate(phi, v_air, dt3);

				        }else if (remoteGlobalPosition.localTimestamp == mavlink_message.timestamp){
				            double timestamp=remoteGlobalPosition.localTimestamp;

				                double dt1 = timestamp - kf_time_old; // calculate dt

				            // predict
				                // [Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);
				                remotePosEstimator.predictEstimate(phi_old, v_air_old, dt1);

				            // udpate position
				                // [Xhat2,P2]  = KF_update_pos(Xhat1,p_pdot',P1,r_a,r_b);
				                remotePosEstimator.updateEstimate(Z);	//execution of update function

				                double dt2 = kf_time - timestamp; // calculate dt

				            // predict
				                // [Xhat_out,omega_out,P]  = KF_predict(Xhat2,v_air,phi,P2,dt2,q,var_phi,gravity);
				                remotePosEstimator.predictEstimate(phi, v_air, dt2);

				        }

				     phi_old = phi;
				     v_air_old = v_air;

	 }else{ //only position measurement is available

	        	double dt1 = remoteGlobalPosition.localTimestamp - kf_time_old; // calculate dt

		    // predict
	        	// [Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);
	        	remotePosEstimator.predictEstimate(phi_old, v_air_old, dt1);

		    // udpate position
	        	// [Xhat2,P2]  = KF_update_pos(Xhat1,p_pdot',P1,r_a,r_b);
	        	remotePosEstimator.updateEstimate(Z);	//execution of update function

		    	double dt2 = kf_time - remoteGlobalPosition.localTimestamp; // calculate dt

		    // predict
		    	// [Xhat_out,omega_out,P]  = KF_predict(Xhat2,v_air_old,phi_old,P2,dt2,q,var_phi,gravity);
		    	remotePosEstimator.predictEstimate(phi_old, v_air_old, dt2);

		    // phi_old_out = phi_old;
		    // v_air_old_out = v_air_old;

	 }


	        //--------------------------------------------------------------------------
			// check if phi measurement is available but no position measurement
			//--------------------------------------------------------------------------
	}else if (mavlink_message.timestamp >= kf_time_old && mavlink_message.timestamp < kf_time){

			    double dt1 = mavlink_message.timestamp - kf_time_old; // calculate dt

			// predict with old phi
			    // [Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);
			    remotePosEstimator.predictEstimate(phi_old, v_air_old, dt1);


			    double dt2 = kf_time - mavlink_message.timestamp; // calculate dt

			// predict with new phi
			    // [Xhat_out,omega_out,P]  = KF_predict(Xhat1,v_air,phi,P1,dt2,q,var_phi,gravity);
			    remotePosEstimator.predictEstimate(phi, v_air, dt2);

			phi_old = phi;
			// v_air_old_out = v_air;

			//--------------------------------------------------------------------------
			// no measurement is available
			//--------------------------------------------------------------------------
	}else {

			    double dt1 = kf_time - kf_time_old; // calculate dt

			    // [Xhat_out,omega_out,P]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);
			    remotePosEstimator.predictEstimate(phi_old, v_air_old, dt1);

			// phi_old_out = phi_old;
			// v_air_old_out = v_air_old;

}

kf_time_old = kf_time;

// transform from NED to ENU
remotePosEstimator.targetEstimatedVel_.x = remotePosEstimator.xhat(3);
remotePosEstimator.targetEstimatedVel_.y = remotePosEstimator.xhat(1);
remotePosEstimator.targetEstimatedVel_.z = -remotePosEstimator.xhat(5);

// update position
remotePosEstimator.targetEstimatedPosLocal_.x = remotePosEstimator.xhat(2);
remotePosEstimator.targetEstimatedPosLocal_.y = remotePosEstimator.xhat(0);
remotePosEstimator.targetEstimatedPosLocal_.z = remotePosEstimator.xhat(4);

// convert back to WGS84
remotePosEstimator.antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,targetEstimatedPosLocal_.y,targetEstimatedPosLocal_.z,targetEstimatedPos_.lat,targetEstimatedPos_.lon,targetEstimatedPos_.alt);

//std::stringstream logmessage;
//logmessage << "predict local pos: " << targetEstimatedPosLocal_ << ", local vel: " << targetEstimatedVel_;
//addLogMessage(vl_DEBUG,logmessage.str());

estimateUpdated=true; //true: new position for motors

// -------------------------------------------------ROMAN-End---------------------------------------------------//

/*
		// update estimator
		if (newTrackedPos) { //check if new position is available
			remotePosEstimator.setNewRemoteGPos(remoteGlobalPosition);
		}

		if (remoteGlobalPosition.localTimestamp - (startTs.tv_sec*1e6 + startTs.tv_nsec*1e-3) < 10*1e6) { // keep estimator running for 10s after last GPOS message
			remotePosEstimator.updateEstimate();	//execution of update function
			estimateUpdated = true;

//			trackingLog.log(vl_DEBUG,"Estimator update done.");
//			printf("azimuth angle: %f deg, elevation angle: %f deg \n",remotePosEstimator.getAzimuth(),remotePosEstimator.getElevation());


		} else {
			if (remoteGlobalPosition.localTimestamp > 0) trackingLog.log(vl_DEBUG,"Not updating estimator anymore");
			else trackingLog.log(vl_DEBUG,"Not yet updating estimator (no global position received so far)");
			estimateUpdated = false;
		}

*/



		/*
		 * call routine of current state
		 */
		motorSetpoints.panCtrltype = ct_undefined;
		motorSetpoints.panValue = 0;
		motorSetpoints.tiltCtrltype = ct_undefined;
		motorSetpoints.tiltValue = 0;
		switch (currentState.get()) {
		case ts_ENDING:
			motorSetpoints.panCtrltype = ct_velocity;
			motorSetpoints.panValue = 0;
			motorSetpoints.tiltCtrltype = ct_velocity;
			motorSetpoints.tiltValue = 0;
			break;
		case ts_INIT:
			if (!commandLineOptions.noMotors) {
				if (motorControl.isInitialized()) {
					if (!motorControl.tiltMotorIsHoming()) {
						motorControl.homeTiltMotor();
						trackingLog.log(vl_INFO, std::string("Homing tilt motor"));
					} else if (motorControl.isTiltHomed()) {
						trackingLog.log(vl_INFO, std::string("Tilt motor homed"));
                        currentState.set(ts_FIND_NORTH);
					}
				}
			} else
				currentState.set(ts_FIND_NORTH);
			break;

		case ts_GPS_TRACKING:
			if(estimateUpdated) {
				gpsTracking.update(curPanAngle,curTiltAngle,motorControl.panPositionReached(),motorControl.tiltPositionReached());
				motorSetpoints = gpsTracking.getNewSetpoints();
			}
			break;

		case ts_FIND_NORTH:
			// process magnetometer reading
			mavlinkMagn.getMag(&localMagn);

			if (findNorth.getLocateState() == fn_NOTREADY) {
				findNorth.init(trackingConfig.findNorth.panSpeed,trackingConfig.findNorth.tiltAngle);
			}
			else if (findNorth.getLocateState() == fn_FINISHED) {
				gpsTracking.setMapping(findNorth.northPanAngleFound(),0);
				currentState.set(ts_GPS_TRACKING);
				findNorth.reset();
			}
			else {
                bool panPosReached = (!commandLineOptions.noMotors) ? motorControl.panPositionReached() : true;
				bool tiltPosReached = (!commandLineOptions.noMotors) ? motorControl.tiltPositionReached() : true;
				findNorth.update(curPanAngle,curTiltAngle,&localMagn,panPosReached,tiltPosReached);
			}
			motorSetpoints = findNorth.getNewSetpoints();

			break;
		case ts_READY:
			motorSetpoints = guiBackend.getNewMotorSetpoints();
			break;

        case ts_LOCATE:
            break;

		case ts_STOP:
			motorSetpoints.panCtrltype = ct_velocity;
			motorSetpoints.panValue = 0;
			motorSetpoints.tiltCtrltype = ct_velocity;
			motorSetpoints.tiltValue = 0;
			if (!commandLineOptions.noMotors) {
				if (motorControl.getTiltSpeed() == 0
						&& motorControl.getPanSpeed() == 0) {
					// Motors have come to a full stop, switch back to mode READY
					motorControl.enablePan();
					motorControl.enableTilt();
					currentState.set(ts_READY);
				}
			}
			break;
		}

		// set output (motor control)
		if (!commandLineOptions.noMotors) {
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
		recorder.record(&startTs, &RSSvalues, motorControl.getPanPosition(), motorControl.getTiltPosition(), &motorSetpoints, &localPosition, &remotePosition, &remotePosEstimator, (int) currentState.get());

		// get log messages
		trackingLog.fetchLogs();

		// signal to backend to receive and send data
		pthread_cond_signal(guiBackend.pDataReadyCond);

		clock_gettime(CLOCK_REALTIME, &curTs);
		waitDuration = (int) (1e6 * (curTs.tv_sec - startTs.tv_sec)
				+ (curTs.tv_nsec - startTs.tv_nsec) / 1000);
		waitDuration = updatePeriod - waitDuration;
		if (waitDuration > 0) {
			usleep(waitDuration);
		}

	}
	/* end tracking antenna */
	guiBackend.killThread();
//	remoteGPS.closeGPSPosServer();
//	GPSSensor.killThread();
	localMavlinkReader.stopReading();
	remoteMavlinkReader.stopReading();
	if (!commandLineOptions.noMotors) {
		motorControl.stop();
	}
	trackingLog.fetchLogs();
	trackingLog.log(vl_INFO, "Tracking Antenna closed");
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
