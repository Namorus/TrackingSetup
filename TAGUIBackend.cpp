/*
 * TAGUIBackend.cpp
 *
 *  Created on: Jun 18, 2013
 *      Author: thomas
 */

#include <netdb.h>      // Needed for the socket functions
#include <sys/poll.h>
#include <sys/socket.h>

#include "trackingsetup/TAGUIBackend.h"

namespace tracking {

TAGUIBackend::TAGUIBackend() :
		pCfg(NULL),
		pCLO(NULL),
		pLog(NULL),
		pLocalPos(NULL),
		pRemotePos(NULL),
		pLocalGps(NULL),
		pRemoteGps(NULL),
		pCurMode(NULL),
		pGPStracking(NULL),
//		pMapEst(NULL),
		pMotorControl(NULL),
		pRecorder(NULL),
		pMotorSetpoints(NULL),
		new_sd(-1),
		serverPort(6542),
		serverThread(0),
		connectionInitialized(false),
		closeServer(false) {

	pDataMutex = new pthread_mutex_t;
	pthread_mutex_init(pDataMutex, NULL);
	pDataReadyCond = new pthread_cond_t;
	pthread_cond_init(pDataReadyCond, NULL);

	pControlMutex = new pthread_mutex_t;
	pthread_mutex_init(pControlMutex, NULL);
	pControlCond = new pthread_cond_t;
	pthread_cond_init(pControlCond, NULL);

	pCloseServerMutex = new pthread_mutex_t;
	pthread_mutex_init(pCloseServerMutex, NULL);

	pConnInitMutex = new pthread_mutex_t;
	pthread_mutex_init(pConnInitMutex, NULL);
}

void* TAGUIBackend::GuiBackendThread(void* arg) {
	TAGUIBackend* self = (TAGUIBackend*) arg;

	/******************
	 *  open socket   *
	 ******************/

	int status;
	struct addrinfo host_info; // The struct that getaddrinfo() fills up with data.
	struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.

	memset(&host_info, 0, sizeof host_info);

	host_info.ai_family = AF_INET;     // IP version 4
	host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP
	host_info.ai_flags = AI_PASSIVE;     // IP Wildcard

	// get address information
	char portNo[6];
	sprintf(portNo, "%d", self->serverPort);
	status = getaddrinfo(NULL, portNo, &host_info, &host_info_list);
	if (status != 0) {
		self->addLogMessage(vl_WARNING,
				std::string("getaddrinfo error: ") + gai_strerror(status));
	}

	int socketfd; // The socket descriptor
	socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype,
			host_info_list->ai_protocol);
	if (socketfd == -1) {
		self->addLogMessage(vl_ERROR,
				std::string("socket error (could not create socket on port ")
						+ portNo + ")");
	}

	// check if socket is already in use and set timeout
	int yes = 1;
	timeval rcvtimeout;
	rcvtimeout.tv_sec = 5;
	rcvtimeout.tv_usec = 0;

	status = setsockopt(socketfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
	status = setsockopt(socketfd, SOL_SOCKET, SO_RCVTIMEO, &rcvtimeout,
			sizeof(rcvtimeout));
	if (status == -1) {
		self->addLogMessage(vl_ERROR, "problem setting rcvtimeout");
	}

	// bind socket
	status = bind(socketfd, host_info_list->ai_addr,
			host_info_list->ai_addrlen);
	if (status == -1) {
		self->addLogMessage(vl_ERROR,
				std::string("Could not bind to socket (port ") + portNo + ")");
	}

	// listen for new connection
	status = listen(socketfd, 1);
	if (status == -1)
		self->addLogMessage(vl_ERROR, "listen error");

//	int new_sd = -1;
	struct sockaddr_storage their_addr;
	socklen_t addr_size = sizeof(their_addr);

	// keep looping until stopped
	bool closing = self->isServerClosing();

	int numRetValues;
	char initString[4];

	char incomming_data_buffer[1000];
	ssize_t bytes_received;

	struct pollfd ufds;
	ufds.events = POLLIN;
	/******************
	 *  listen loop   *
	 ******************/

	while (!closing) {
		/* wait for first connection */
		// accept waits the amount of time set with rcvtimeout for a new connection
		self->new_sd = accept(socketfd, (struct sockaddr *) &their_addr,
				&addr_size);
		if (self->new_sd != -1) {
			self->addLogMessage(vl_INFO,
					"accepted new connection from GUI client");
		}
		// set default value: do not stream log to GUI -> has to be activated when a new connection is made
		self->pLog->setCacheLogForGui(false);
		// keep looping until stopped
		closing = self->isServerClosing();

		while (!closing && self->new_sd != -1) {
			if (!self->isConnectionInitialized()) {
				// receiving data
				bytes_received = recv(self->new_sd, incomming_data_buffer, 1000,
						0);

				if (bytes_received == 0) {
					// host closed connection, so we close it too.
					// -> will fall back to outer while loop and wait for new connection
					close(self->new_sd);
					self->new_sd = -1;
				} else if (bytes_received == -1) {
					// there is a problem with the connection, so we close it.
					// -> will fall back to outer while loop and wait for new connection
					close(self->new_sd);
					self->new_sd = -1;
				} else {
//					self->addLogMessage(vl_DEBUG,std::string("new data from remote GUI received: ") + incomming_data_buffer);
					incomming_data_buffer[bytes_received] = '\0';
					numRetValues = sscanf(incomming_data_buffer,
							"$TrackingAntennaGUI %4s", initString);
					if (numRetValues == 1
							&& std::string(initString).compare("INIT") == 0) {
						self->addLogMessage(vl_DEBUG,
								"GUI client request accepted");
						self->setConnectionInitialized(true);
						self->sendConfig();
					} else {
						close(self->new_sd);
						self->new_sd = -1;
						self->setConnectionInitialized(false);
					}
				}
			} else {
//				self->addLogMessage(vl_DEBUG,std::string("waiting to send data to GUI client"));
				pthread_mutex_lock(self->pDataMutex);
				pthread_cond_wait(self->pDataReadyCond, self->pDataMutex);
				self->sendData();
				pthread_mutex_unlock(self->pDataMutex);
				ufds.fd = self->new_sd;
				status = poll(&ufds, 1, 5000); // wait for 5 seconds (timeout)
				if (status == -1) {
					//something went wrong while using poll
					self->addLogMessage(vl_DEBUG,
							std::string(
									"error using poll while waiting for data"));
					close(self->new_sd);
					self->new_sd = -1;
					self->setConnectionInitialized(false);
				} else if (status == 0) {  //timeout
					self->addLogMessage(vl_DEBUG,
							std::string(
									"timeout reached while waiting for data from GUI"));
					close(self->new_sd);
					self->new_sd = -1;
					self->setConnectionInitialized(false);
				} else { // receive data
					bzero(incomming_data_buffer, 1000);
					bytes_received = recv(self->new_sd, incomming_data_buffer,
							1000, 0);
					incomming_data_buffer[bytes_received] = '\0';
					if (bytes_received > 0) {
//						self->addLogMessage(vl_DEBUG,std::string("received data from GUI client"));
						self->readData(std::string(incomming_data_buffer));
					} else {
						close(self->new_sd);
						self->new_sd = -1;
						self->setConnectionInitialized(false);
					}
				}

			}

			/* check if Server should be closed */
			closing = self->isServerClosing();
		}

	}

	close(self->new_sd);
	close(socketfd);
	return 0;

}

int TAGUIBackend::startThread(int _serverPort) {
	serverPort = _serverPort;
	char portNo[6];
	sprintf(portNo, "%d", serverPort);
	addLogMessage(vl_INFO,
			std::string("starting GUI backend server on port ") + portNo);
	return pthread_create(&serverThread, NULL, GuiBackendThread, (void*) this);
}

void TAGUIBackend::killThread() {
	pthread_cond_signal(pDataReadyCond);
	pthread_mutex_lock(pCloseServerMutex);
	closeServer = true;
	pthread_mutex_unlock(pCloseServerMutex);

	pthread_join(serverThread, NULL);
}

//setpoint TAGUIBackend::getNewMotorSetpoint() {
//	setpoint temp;
//	temp = remoteSetpoint;
//	remoteSetpoint.type = ct_undefined;
//	remoteSetpoint.pan = 0;
//	remoteSetpoint.tilt = 0;
//	return temp;
//}

setpoints TAGUIBackend::getNewMotorSetpoints() {
	setpoints temp = remoteSetpoints;
	remoteSetpoints.panCtrltype = ct_undefined;
	remoteSetpoints.panValue = 0;
	remoteSetpoints.tiltCtrltype = ct_undefined;
	remoteSetpoints.tiltValue = 0;
	return temp;
}

void TAGUIBackend::sendConfig() {
	int n = 0;
	std::string data;
	if (pCfg != NULL) {
		sendBuffer.str("");
		sendBuffer << *pCfg;
		data = sendBuffer.str();
		n = write(new_sd, data.c_str(), data.size());
	}
	if (n > 0) {
		std::stringstream logmsg;
		logmsg << "Config sent to GUI client";
//		logmsg << " (" << n << " bytes sent): " << data;

		addLogMessage(vl_DEBUG, logmsg.str());
	}
}
void TAGUIBackend::sendData() {
	if (pCLO != NULL) {
		sendBuffer.str("");
		// parse data
		sendBuffer << "$localPos " << (*pLocalPos);
		if (!pCLO->noLocalGPS) {
			sendBuffer << (*pLocalGps);
		}

		sendBuffer << "$remotePos " << (*pRemotePos);
		if (!pCLO->noRemoteGPS) {
			sendBuffer << (*pRemoteGps);
		}

		if (!pCLO->noMotors)
			sendBuffer << (*pMotorControl);

		sendBuffer << (*pCurMode);

		sendBuffer << (*pMotorSetpoints);

//		sendBuffer << (*pMapEst); //TODO: send FindNorth instead

		sendBuffer << (*pLog);
		pLog->clearCacheForGui();

		std::string data = sendBuffer.str();
		int n = write(new_sd, data.c_str(), data.size());
//		std::stringstream logmsg;
//		logmsg << n << " bytes sent to GUI client: " << data;
//		addLogMessage(vl_DEBUG,logmsg.str());
	}

}

void TAGUIBackend::readData(std::string data) {
	std::istringstream dataStream;
	dataStream.str(data);

	// TODO: read out data
//	addLogMessage(vl_DEBUG,"received: " + data);
	// simple check if stream is valid
	while (!dataStream.eof()) {
		char nextChar = dataStream.get();
		if (nextChar == '$') {
			std::string type;
			dataStream >> type;
			// Data received is a new configuration
			if (type.compare("Config") == 0) {
				addLogMessage(vl_INFO, "received new config");
				TAConfig* pNewCfg = new TAConfig;
				dataStream >> *pNewCfg;
				// newCfg->display();
				pCfg->setNewConfig(pNewCfg);
			}
			// Data received is a new mode
			else if (type.compare("Mode") == 0) {
				addLogMessage(vl_DEBUG, "received new mode");
				int newMode;
				dataStream >> newMode;
				pCurMode->set((trackingMode) newMode);
			}
			// Data received is a new setpoint
			else if (type.compare("Setpoints") == 0) {
				addLogMessage(vl_DEBUG, "received a new setpoint");
				int temp;
				dataStream >> temp;
				remoteSetpoints.panCtrltype = (TActrlType) temp;
				dataStream >> remoteSetpoints.panValue;
				dataStream >> temp;
				remoteSetpoints.tiltCtrltype = (TActrlType) temp;
				dataStream >> remoteSetpoints.tiltValue;
			}

			// Data received is a new configuration for the GUI and this backend
			// At the moment config allows to set if log is send to GUI or not.
			// -> is disabled, as soon the connection is lost in order to avoid the buffer to grow immensely
			else if (type.compare("guiconfig") == 0) {
				int getLog;
				dataStream >> getLog;
				pLog->setCacheLogForGui((bool) getLog);
			}

			// Data received is a request to start recording
			else if (type.compare("startrecording") == 0) {
				recorderSettings recSettings;
				dataStream >> recSettings;
				pRecorder->start(recSettings);
			} else if (type.compare("stoprecording") == 0) {
				pRecorder->stop();
			} else if (type.compare("doManualMapping") == 0) {
				//TODO: implement this again!
//				pMapEst->setMapping(pMotorControl->getPanAngle(),pMotorControl->getTiltAngle());
//				pGPStracking->setMapping(pMapEst->getPanOffset(),pMapEst->getTiltOffset());
			} else if (type.compare("manualObjPos") == 0) {
                dataStream >> *pRemotePos;
//				GPSPos* pObjPos = new GPSPos;
//				dataStream >> *pObjPos;
//                pRemotePos->newManualPos(pObjPos);
//				delete pObjPos;
			} else if (type.compare("manualAntPos") == 0) {
				dataStream >> *pLocalPos;
			}
		} else if (nextChar == ' ' || nextChar == '\0') {
			//
		} else {
			addLogMessage(vl_WARNING,
					"could not parse data received from GUI client!");
		}
	}
//	if(atoi(dataBuffer.c_str()) == 15) {
//		pCurMode->set(ENDING);
//	}
	// wait for the main thread to be ready to for new control data
//	pthread_mutex_lock(pControlMutex);
//	pthread_cond_wait(pControlCond,pControlMutex);
	// TODO: set data

//	pthread_mutex_unlock(pControlMutex);

}

bool TAGUIBackend::isServerClosing() {
	bool temp;
	pthread_mutex_lock(pCloseServerMutex);
	temp = closeServer;
	pthread_mutex_unlock(pCloseServerMutex);

	return temp;

}

bool TAGUIBackend::isConnectionInitialized() {
	bool temp;
	pthread_mutex_lock(pConnInitMutex);
	temp = connectionInitialized;
	pthread_mutex_unlock(pConnInitMutex);

	return temp;
}

void TAGUIBackend::setConnectionInitialized(bool setVal) {
	pthread_mutex_lock(pConnInitMutex);
	connectionInitialized = setVal;
	pthread_mutex_unlock(pConnInitMutex);
}

void TAGUIBackend::setCfg(TAConfig* cfg) {
	pCfg = cfg;
}

void TAGUIBackend::setClo(commandLineOptions* clo) {
	pCLO = clo;
}

void TAGUIBackend::setCurMode(TAmode* curMode) {
	pCurMode = curMode;
}

void TAGUIBackend::setLocalPos(GPSPos* localPos) {
	pLocalPos = localPos;
}

void TAGUIBackend::setRemotePos(GPSPos* remotePos) {
	pRemotePos = remotePos;
}

void TAGUIBackend::setLocalGps(TSmavlinkGPS* localGps) {
	pLocalGps = localGps;
}

void TAGUIBackend::setRemoteGps(TSmavlinkGPS* remoteGps) {
	pRemoteGps = remoteGps;
}
void TAGUIBackend::setGpStracking(TAGPSTracking* gpStracking) {
	pGPStracking = gpStracking;
}

void TAGUIBackend::setLog(TALogger* log) {
	pLog = log;
}

void TAGUIBackend::setMotorControl(TAMotorControl* motorControl) {
	pMotorControl = motorControl;
}

void TAGUIBackend::setMotorSetpoints(setpoints* motorSetpoints) {
	pMotorSetpoints = motorSetpoints;
}

void TAGUIBackend::setRecorder(TARecorder* recorder) {
	pRecorder = recorder;
}

//void TAGUIBackend::setTAclasses(TAConfig* _pCfg,
//								commandLineOptions* _pCLO,
//								TALogger* _pLog,
//								TSPosInput* _pLocalPos,
//								TSPosInput* _pRemotePos,
//								TAmode* _pCurMode,
//								TAGPSTracking* _pGPStracking,
//								TAMappingEstimation* _pMapEst,
//								TAMotorControl* _pMotorControl,
//								setpoints* _pMotorSetpoints,
//								GPSPos* _pAntPos) {
//	pCfg = _pCfg;
//	pCLO = _pCLO;
//	pLog = _pLog;
//	pLocalPos = _pLocalPos;
//	pRemotePos = _pRemotePos;
//	pCurMode = _pCurMode;
////	pRSStracking = _pRSStracking;
//	pGPStracking = _pGPStracking;
//	pMapEst = _pMapEst;
//	pMotorControl = _pMotorControl;
//	pMotorSetpoints = _pMotorSetpoints;
//	pAntPos = _pAntPos;
//}

} /* namespace tracking */
