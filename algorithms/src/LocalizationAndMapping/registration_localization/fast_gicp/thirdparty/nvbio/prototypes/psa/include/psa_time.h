/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_time
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface timming definitions for CPU.
 */

#ifndef PSA_TIME_H_
#define PSA_TIME_H_

#include <time.h>
#include <sys/time.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

double sampleTime();

#endif /* PSA_TIME_H_ */