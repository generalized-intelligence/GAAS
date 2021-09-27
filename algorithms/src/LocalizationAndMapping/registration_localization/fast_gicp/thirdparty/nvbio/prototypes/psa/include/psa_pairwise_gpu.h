/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_pairwise_gpu
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface pairwise definitions for basic operations exlusive for GPU. 
 */


#ifndef PSA_PAIRWISE_GPU_H_
#define PSA_PAIRWISE_GPU_H_

#include "psa_commons.h"
#include "psa_sequences.h"
#include "psa_alignments.h"

psaError_t localProcessPairwiseReference(sequences_t* references, sequences_t *candidates, sequences_t *queries, alignments_t *alignments);
psaError_t processPairwiseStream(sequences_t *candidates, sequences_t *queries, alignments_t *alignments);

#endif /* PSA_PAIRWISE_GPU_H_ */
