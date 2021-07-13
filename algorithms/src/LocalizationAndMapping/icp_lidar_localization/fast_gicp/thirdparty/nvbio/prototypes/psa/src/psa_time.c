/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_time
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface timming definitions for CPU.
 */

#include "../include/psa_time.h"

double sampleTime()
{
	struct timespec tv;

	#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	tv.tv_sec = mts.tv_sec;
	tv.tv_nsec = mts.tv_nsec;

	#else
	clock_gettime(CLOCK_REALTIME, &tv);
	#endif

	return((tv.tv_sec+tv.tv_nsec/1000000000.0));
}


