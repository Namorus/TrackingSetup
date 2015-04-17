/*
 * TAGUIBackend.h
 *
 *  Created on: Jun 18, 2013
 *      Author: thomas
 */

#ifndef TAGUIBACKEND_H_
#define TAGUIBACKEND_H_


#include <trackingsetup/config.h>
#include <trackingsetup/gps_tracking.h>
#include <trackingsetup/logger.h>
#include <trackingsetup/mavlink_gps.h>
#include <trackingsetup/motor_control.h>
#include <trackingsetup/recorder.h>
#include <trackingsetup/tracking_state.h>
#include <trackingsetup/trackingsetup.h>
#include <trackingsetup/types.h>

namespace tracking {

class GuiBackend: public tracking::TrackingSetup {
public:
	pthread_mutex_t* pDataMutex;
	pthread_cond_t* pDataReadyCond;

	pthread_mutex_t* pControlMutex;
	pthread_cond_t* pControlCond;

	GuiBackend();

	static void* GuiBackendThread(void* arg);

	int startThread(int _serverPort);

	void killThread();

//	setpoint getNewMotorSetpoint();

	setpoints getNewMotorSetpoints();

	void sendConfig();

	void sendData();

	void readData(std::string dataBuffer);

	bool isServerClosing();

	bool isConnectionInitialized();

	void setConnectionInitialized(bool setVal);

//	void setTAclasses(TAConfig* _pCfg,
//					  commandLineOptions* _pCLO,
//					  TALogger* _pLog,
//					  TSPosInput* _pLocalPos,
//					  TSPosInput* _pRemotePos,
//			          TAmode* _pCurMode,
//			          TAGPSTracking* _pGPStracking,
//			          TAMappingEstimation* _pMapEst,
//			          TAMotorControl* _pMotorControl,
//			          setpoints* _pMotorSetpoints,
//			          GPSPos* _pAntPost);
	void setLocalPos(GPSPos* localPos);
	void setRemotePos(GPSPos* remotePos);
	void setLocalGps(MavlinkGps* localGPS);
	void setRemoteGps(MavlinkGps* remoteGPS);
	void setConfig(Config* cfg);
	void setCommandLineOptions(commandLineOptions* clo);
	void setCurMode(State* curMode);
	void setGpsTracking(GpsTrackingMode* gpStracking);
	void setLog(Logger* log);
	void setMotorControl(MotorControl* motorControl);
	void setMotorSetpoints(setpoints* motorSetpoints);
	void setRecorder(Recorder* recorder);

	template<class T> void addToData(T* ptr) {
		sendBuffer << *ptr;
	}

private:
	Config* pCfg;
	commandLineOptions* pCLO;
	Logger* pLog;
	GPSPos* pLocalPos;
	GPSPos* pRemotePos;
	MavlinkGps* pLocalGps;
	MavlinkGps* pRemoteGps;
	State* pCurMode;
	GpsTrackingMode* pGPStracking;
	MotorControl* pMotorControl;
	Recorder* pRecorder;

	setpoints* pMotorSetpoints;

//	GPSPos* pAntPos;

	int new_sd;

	int serverPort;

	std::ostringstream sendBuffer;

	pthread_t serverThread;

//	setpoint remoteSetpoint;

	setpoints remoteSetpoints;

	bool connectionInitialized;
	pthread_mutex_t* pConnInitMutex;

	bool closeServer;
	pthread_mutex_t* pCloseServerMutex;

};

} /* namespace tracking */
#endif /* TAGUIBACKEND_H_ */
