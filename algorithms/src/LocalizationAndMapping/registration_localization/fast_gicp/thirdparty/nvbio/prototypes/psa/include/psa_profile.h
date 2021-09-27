/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_profile
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Process functions for GEM profile files.
 */

#ifndef PSA_PROFILE_H_
#define PSA_PROFILE_H_

#include <string.h>
#include "psa_commons.h"
#include "psa_errors.h"

psaError_t countCandidates(char* cadena, uint32_t *numberOfCandidates, uint32_t *querySize);
psaError_t countRegionsProfile(FILE *fp, float distance, 
								uint32_t *retNumQueries, uint32_t *retTotalSizeQueries, 
								uint32_t *retNumCandidates, uint32_t *retTotalSizeCandidates);
psaError_t processQuery(uint32_t queryNumber, char *textLine,
				 		   char *queries, uint32_t *h_PositionCandidates, uint32_t *h_AlignmentInfo, uint32_t *listResults,
				 		   uint32_t *retSizeQueries, uint32_t *retNumCandidates);
psaError_t loadRegionsProfile(FILE *fp, char *h_ASCIIQueries, uint32_t *h_ASCIISizeQueries, uint32_t *h_ASCIIPositionQueries,
								uint32_t *h_GenomePositionCandidates, uint32_t *h_AlignmentInfo, uint32_t *h_AlignmentScores);


#endif /* BP_PROFILE_H_ */













