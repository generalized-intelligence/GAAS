/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_regions
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface to process GEM regions.
 */


#ifndef PSA_REGIONS_H_
#define PSA_REGIONS_H_

#include "psa_commons.h"
#include "psa_sequences.h"
#include "psa_alignments.h"
#include "psa_errors.h"


typedef struct {
	float 			distance;
	uint32_t 		averageSizeQuery;
	uint32_t 		averageCandidatesPerQuery;
	sequences_t 	*queries;
	sequences_t 	*candidates;
	alignments_t 	*alignments;
} regions_t;

psaError_t initRegions(regions_t **regions, uint32_t numQueries, uint32_t numCandidates, uint32_t numAlignments);
psaError_t allocateRegions(regions_t *regions, float distance, 
							  uint32_t totalSizeQueries, seq_format_t QUERIES_ASCII, 
							  uint32_t totalSizeCandidates, seq_format_t CANDIDATES_ASCII);
psaError_t loadRegions(FILE *fp, regions_t *regions);
psaError_t saveRegions(FILE *fp, regions_t *regions);
psaError_t statistics(regions_t *regions);
psaError_t printRegions(regions_t *regions, uint32_t numAlignments, uint32_t reformated);
psaError_t freeRegions(regions_t **regions);

#endif /* PSA_REGIONS_H_ */











