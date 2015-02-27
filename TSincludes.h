/*
 * TAincludes.h
 *
 *  Created on: Jun 15, 2013
 *      Author: thomas
 */

#ifndef TSINCLUDES_H_
#define TSINCLUDES_H_

#include <cstdio>
#include <cstdlib>
#include <stdint.h>
#include <iostream>
#include <cstring>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <locale>
#include <ctime>
#include <vector>
#include <list>
#include <sys/socket.h> // Needed for the socket functions
#include <sys/poll.h>
#include <netdb.h>      // Needed for the socket functions
#include <unistd.h>
#include <algorithm>

#include <common/mavlink.h>

#include "TAtypes.h"
#include "TAClass.h"
#include "TALogger.h"
#include "TAConfig.h"
#include "TSmavlinkReader.h"
#include "EposComm.h"
#include "Epos.h"
#include "TAMotorControl.h"
#include "TATrackingMode.h"
#include "TAmode.h"
#include "TAMappingEstimation.h"
#include "TAGPSTracking.h"
#include "TAMotorControl.h"
#include "TARecorder.h"
#include "TSfindNorth.h"
#include "TSmavlinkGPS.h"
#include "TSmavlinkMag.h"
#include "TSmavlinkRadioStatus.h"
#include "TAGUIBackend.h"


#endif /* TSINCLUDES_H_ */
