/*
 * TSmavlinkMag.cpp
 *
 *  Created on: Feb 4, 2015
 *      Author: asl
 */

#include "trackingsetup/TAtypes.h"
#include "trackingsetup/TSmavlinkRadioStatus.h"

namespace tracking {

TSmavlinkRadioStatus::TSmavlinkRadioStatus(MavlinkMessages* mavlinkMessages) :
    mavlinkMessages_(mavlinkMessages),
    lastTimestamp_(0) {

}

bool TSmavlinkRadioStatus::getRSSI(RadioRSSI* radioRSSI) {
    if(mavlinkMessages_->lastRadioStatus > lastTimestamp_) {
        radioRSSI->rssi = mavlinkMessages_->radio_status.rssi;
        radioRSSI->remRssi = mavlinkMessages_->radio_status.remrssi;
        radioRSSI->txbuf = mavlinkMessages_->radio_status.txbuf;
        radioRSSI->noise = mavlinkMessages_->radio_status.noise;
        radioRSSI->remNoise = mavlinkMessages_->radio_status.remnoise;
        radioRSSI->rxErrors = mavlinkMessages_->radio_status.rxerrors;
        radioRSSI->rxErrors = mavlinkMessages_->radio_status.fixed;
        lastTimestamp_ = mavlinkMessages_->lastRadioStatus;
        return true;
    }

    return false;

}

TSmavlinkRadioStatus::~TSmavlinkRadioStatus() {
	// TODO Auto-generated destructor stub
}

} /* namespace tracking */
