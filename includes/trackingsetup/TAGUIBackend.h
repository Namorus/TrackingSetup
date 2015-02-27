/*
 * TAGUIBackend.h
 *
 *  Created on: Jun 18, 2013
 *      Author: thomas
 */

#ifndef TAGUIBACKEND_H_
#define TAGUIBACKEND_H_

#include "TSincludes.h"
#include "TAClass.h"

namespace tracking {

class TAGUIBackend: public tracking::TAClass {
public:
	pthread_mutex_t* pDataMutex;
	pthread_cond_t* pDataReadyCond;

	pthread_mutex_t* pControlMutex;
	pthread_cond_t* pControlCond;

	TAGUIBackend();

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
	void setLocalGps(TSmavlinkGPS* localGPS);
	void setRemoteGps(TSmavlinkGPS* remoteGPS);
	void setCfg(TAConfig* cfg);
	void setClo(commandLineOptions* clo);
	void setCurMode(TAmode* curMode);
	void setGpStracking(TAGPSTracking* gpStracking);
	void setLog(TALogger* log);
	void setMotorControl(TAMotorControl* motorControl);
	void setMotorSetpoints(setpoints* motorSetpoints);
	void setRecorder(TARecorder* recorder);

	template<class T> void addToData(T* ptr) {
		sendBuffer << *ptr;
	}

private:
	TAConfig* pCfg;
	commandLineOptions* pCLO;
	TALogger* pLog;
	GPSPos* pLocalPos;
	GPSPos* pRemotePos;
	TSmavlinkGPS* pLocalGps;
	TSmavlinkGPS* pRemoteGps;
	TAmode* pCurMode;
	TAGPSTracking* pGPStracking;
	TAMotorControl* pMotorControl;
	TARecorder* pRecorder;

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
