/*
 * current_utc_time.h
 *
 *  Created on: May 5, 2015
 *      Author: Maverick
 */

#ifndef CURRENT_UTC_TIME_H_
#define CURRENT_UTC_TIME_H_

#include <time.h>
#include <sys/time.h>
#include <stdio.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

void current_utc_time(struct timespec *ts);

#endif /* CURRENT_UTC_TIME_H_ */
