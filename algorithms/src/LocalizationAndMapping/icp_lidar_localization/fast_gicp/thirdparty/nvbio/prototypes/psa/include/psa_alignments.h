/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_alignments
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API functions to manage the alignments (sink and score)
 */

#ifndef PSA_ALIGNMENTS_H_
#define PSA_ALIGNMENTS_H_

#include "psa_commons.h"
#include "psa_errors.h"
#include "psa_alignments.h"


typedef uint32_t alignmentInfo_t;

typedef struct {
	uint32_t column;
	int32_t score;
} alignmentEntry_t;

typedef struct {
	uint32_t num;
 	uint32_t numReordered;
 	/* aligment structures */
	alignmentInfo_t		*h_info;
	alignmentInfo_t		*d_info;
	alignmentEntry_t	*h_results;
	alignmentEntry_t	*d_results;
 	/* reorder structures */
	alignmentEntry_t	*h_reorderResults;
	alignmentEntry_t	*d_reorderResults;
} alignments_t;

psaError_t initAlignments(alignments_t **alignments, uint32_t numAlignments);
psaError_t allocateAlignments(alignments_t *alignments);
psaError_t loadAlignments(FILE *fp, alignments_t *alignments);
psaError_t saveAlignments(FILE *fp, alignments_t *alignments);
psaError_t saveASCIIAlignments(FILE *fp, alignments_t *alignments);
psaError_t freeAlignments(alignments_t **alignments);


#endif /* PSA_ALIGNMENTS_H_ */
