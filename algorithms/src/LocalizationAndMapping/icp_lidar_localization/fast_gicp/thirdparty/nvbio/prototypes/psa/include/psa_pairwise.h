/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_pairwise
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface pairwise definitions for basic operations. 
 */

#ifndef PSA_PAIRWISE_H_
#define PSA_PAIRWISE_H_

#include "psa_commons.h"
#include "psa_regions.h"
#include "psa_sequences.h"
#include "psa_alignments.h"


char* buildTag();
psaError_t printBuildInfo();
psaError_t transformDataReferences(sequences_t *references);
psaError_t transformDataQueries(sequences_t *queries);
psaError_t transformDataCandidates(sequences_t *candidates);
psaError_t processPairwiseReference(sequences_t* references, sequences_t *candidates, sequences_t *queries, alignments_t *alignments);
psaError_t processPairwiseStream(sequences_t *candidates, sequences_t *queries, alignments_t *alignments);

#ifdef CUDA
psaError_t transferGPUtoCPU(alignments_t *alignments);
psaError_t transferCPUtoGPUReference(sequences_t* references, sequences_t *queries, sequences_t *candidates, alignments_t *alignments);
psaError_t transferCPUtoGPUStream(sequences_t *queries, sequences_t *candidates, alignments_t *alignments);
#endif


#endif /* PSA_PAIRWISE_H_ */
