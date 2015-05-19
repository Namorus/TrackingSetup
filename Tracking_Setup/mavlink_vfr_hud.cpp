/*
 * mavlink_vfr_hud.cpp
 *
 *  Created on: May 17, 2015
 *      Author: Roman Meier
 */


#include <sys/time.h>

#include <trackingsetup/mavlink_vfr_hud.h>

namespace tracking {

MavlinkVfrHud::MavlinkVfrHud(MavlinkMessages* mavlinkMessages) :
        mavlinkMessages_(mavlinkMessages), lastTimestamp_(0) {
}

MavlinkVfrHud::~MavlinkVfrHud() {
	// TODO Auto-generated destructor stub
}

bool MavlinkVfrHud::getVfrHud(VfrHud* vfr_hud) {

    if (mavlinkMessages_->lastVfrHud > lastTimestamp_) {
    	vfr_hud->airspeed = mavlinkMessages_-> vfrhud_mavlink.airspeed; ///< Current airspeed in m/s
    	vfr_hud->groundspeed = mavlinkMessages_-> vfrhud_mavlink.groundspeed; ///< Current ground speed in m/s
    	vfr_hud->alt = mavlinkMessages_-> vfrhud_mavlink.alt; ///< Current altitude (MSL), in meters
    	vfr_hud->climb = mavlinkMessages_-> vfrhud_mavlink.climb; ///< Current climb rate in meters/second
    	vfr_hud->heading = mavlinkMessages_-> vfrhud_mavlink.heading; ///< Current heading in degrees, in compass units (0..360, 0=north)
    	vfr_hud->throttle = mavlinkMessages_-> vfrhud_mavlink.throttle; ///< Current throttle setting in integer percent, 0 to 100

    //	vfr_hud->timestamp = mavlinkMessages_->vfrhud_mavlink.time_boot_ms;

        // handle timestamps
        lastTimestamp_ = mavlinkMessages_->lastVfrHud;		//needed to check if new message is available
        vfr_hud->localTimestamp = mavlinkMessages_->lastVfrHud;	//save lacalTimestamp (time, message has been received by antenna)

		return true;
	}
	return false;
}


} /* namespace tracking */



