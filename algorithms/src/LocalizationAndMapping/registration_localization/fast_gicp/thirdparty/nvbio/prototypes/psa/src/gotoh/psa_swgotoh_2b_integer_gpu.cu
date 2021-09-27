/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_swgotoh_2b_integer_gpu
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Device functions for the SW-Gotoh GPU implementation.
 *				(A) Which uses the 32bit integer resources and store the temporal columns with 8 bits.
 *				(B) The bases are represented using 2 bits/base.
 */

extern "C" {
#include "../../include/psa_pairwise_gpu.h"
}
#include <cuda_runtime.h>
#include <cuda.h>

#ifndef QUERIES_SIZE
	#define QUERIES_SIZE 		100
#endif

#ifndef CANDIDATES_SIZE
	#define CANDIDATES_SIZE 	120
#endif

#define MAX3(a,b,c)				(MAX(MAX(a, b), c))

#define WARP_SIZE				32
#define MAX_THREADS_PER_SM		64
#define CUDA_NUM_THREADS		64

#define THREADS_PER_SEGMENT		32
#define NUM_SW_PER_BLOCK		(MAX_THREADS_PER_SM / THREADS_PER_SEGMENT)
#define NUM_WARPS				(MAX_THREADS_PER_SM / WARP_SIZE)

#define BAND_LEN			 	8
#define MAX_QUERY_SIZE			QUERIES_SIZE
#define RAW_BASES_PER_ENTRY		(UINT32_LENGTH / RAW_4B_LENGTH)

typedef int32_t	score_type;
//typedef char2	score_type2;
//typedef int2	score_type2;
typedef short2	score_type2;

inline __device__
void update_band(int32_t idRow, char q_i, char *ref_cache, score_type *H_band, score_type *F_band,
				score_type2 *temp, score_type *H_maxScore,
				const score_type MATCH_SCORE, const score_type MISMATCH_SCORE,
				const score_type OPEN_INDEL_SCORE, const score_type EXTEND_INDEL_SCORE)
{
	score_type H_diag   = H_band[0];
	H_band[0] 		  	= temp[idRow].x;
	score_type E 	    = temp[idRow].y;

    #pragma unroll
    for (uint32_t j = 1; j <= BAND_LEN; ++j)
    {
        // update F
        const score_type ftop = F_band[j] + EXTEND_INDEL_SCORE;
        const score_type htop = H_band[j] + OPEN_INDEL_SCORE;
        F_band[j] = MAX(ftop, htop);

        // update E
        const score_type eleft = E + EXTEND_INDEL_SCORE;
        const score_type hleft = H_band[j-1] + OPEN_INDEL_SCORE;
        E = MAX(eleft, hleft);

        const char r_j            = ref_cache[j-1];
        const score_type diagonal = (r_j == q_i) ? H_diag + MATCH_SCORE : H_diag + MISMATCH_SCORE;
        const score_type top      = F_band[j];
        const score_type left     = E;
        	  score_type hi       = MAX3(left, top, diagonal);
                         hi       = MAX(hi, 0);
        H_diag                    = H_band[j];
        H_band[j]                 = hi;
        (*H_maxScore) = MAX((*H_maxScore), hi);
    }

    // save the last entry of the band
    temp[idRow]  = make_short2(H_band[BAND_LEN], E);
}

__global__
void localProcessSWTiling(RAWHlfEntry_t *d_CandidatesHlfRaw, uint32_t *d_CandidatesHlfRAWposition,
		   RAWHlfEntry_t *d_QueriesHlfRaw, uint32_t *d_QueriesHlfRAWposition,
		   alignmentInfo_t *d_AlignmentsInfo, alignmentEntry_t *d_AlignmentsResults,
		   uint32_t querySize, uint32_t candidateSize, uint32_t candidatesNum)
{
	const uint32_t idCandidate  = blockIdx.x * MAX_THREADS_PER_SM + threadIdx.x;

	if (idCandidate < candidatesNum)
	{
		const RAWHlfEntry_t* candidate = d_CandidatesHlfRaw + d_CandidatesHlfRAWposition[idCandidate];
		const RAWHlfEntry_t* query	= d_QueriesHlfRaw + d_QueriesHlfRAWposition[d_AlignmentsInfo[idCandidate]];

		// All the scores have to be absolute numbers:
		// Original BWA Scores: Gap_Ex = -1; Gap_Op = -2; Match = 2; Miss = -5;
		const score_type   MATCH_SCORE			=	 2;
		const score_type   MISMATCH_SCORE 		=	-5;
		const score_type   OPEN_INDEL_SCORE		=	-2;
		const score_type   EXTEND_INDEL_SCORE	=	-1;
		const score_type   ZERO					=	 0;

		char    		r_cache[BAND_LEN];
		score_type2 	temp   [MAX_QUERY_SIZE];
		score_type 		H_band [BAND_LEN + 1];
		score_type 		F_band [BAND_LEN + 1];

		const int32_t numRows = querySize, numColumns = candidateSize;
		int32_t idColumn, idRow, idBand;
		score_type H_maxScore = ZERO;

		uint32_t entryCandidate, idCandidateEntry = 0;

		for(idBand = 0; idBand < MAX_QUERY_SIZE; ++idBand){
			temp[idBand].x = ZERO;
			temp[idBand].y = ZERO;
		}

		// Compute Score SW-GOTOH
		for(idColumn = 0; idColumn < numColumns; idColumn += BAND_LEN){
			uint32_t entryQuery, idQueryEntry = 0;
			if((idColumn % RAW_BASES_PER_ENTRY) == 0){
				entryCandidate = candidate[idCandidateEntry];
				idCandidateEntry++;
			}

	        // Load a block of entries from the reference
	        #pragma unroll
	        for (uint32_t idBand = 0; idBand < BAND_LEN; ++idBand){
	            r_cache[idBand] = entryCandidate & 0x3;
	        	entryCandidate >>= RAW_4B_LENGTH;
	        }

	        // Initialize the first band
	        #pragma unroll
	        for (uint32_t idBand = 0; idBand <= BAND_LEN; ++idBand){
	        	H_band[idBand] = ZERO;
	        	F_band[idBand] = ZERO;
	        }

			#pragma unroll 4
	        for(idRow = 0; idRow < numRows; ++idRow){
	        	entryQuery >>= RAW_4B_LENGTH;
				if((idRow % RAW_BASES_PER_ENTRY) == 0){
					entryQuery = query[idQueryEntry];
					idQueryEntry++;
				}

				update_band(idRow, entryQuery & 0x03, r_cache, H_band, F_band, temp, &H_maxScore, 
						   MATCH_SCORE, MISMATCH_SCORE, OPEN_INDEL_SCORE, EXTEND_INDEL_SCORE);
	        }
		}
		d_AlignmentsResults[idCandidate].score  	= H_maxScore;
		d_AlignmentsResults[idCandidate].column     = 0;
	}
}


extern "C"
psaError_t localProcessPairwiseStream(sequences_t *candidates, sequences_t *queries, alignments_t *alignments)
{
	uint32_t blocks = DIV_CEIL(candidates->num, CUDA_NUM_THREADS);
	uint32_t threads = CUDA_NUM_THREADS;
	uint32_t querySize = queries->h_size[0];
	uint32_t candidateSize = candidates->h_size[0];

	cudaThreadSetCacheConfig(cudaFuncCachePreferL1);

	printf("Grid Size: %d, Block Size: %d, Total alignments: %d, BAND_LEN: %d \n", blocks, threads, candidates->num, BAND_LEN);
	localProcessSWTiling<<<blocks, threads>>>(candidates->d_HlfRAW, candidates->d_HlfRAWposition,
								 			queries->d_HlfRAW, queries->d_HlfRAWposition,
											alignments->d_info, alignments->d_results,
								 			querySize, candidateSize, candidates->num);

	cudaThreadSynchronize();

	return (SUCCESS);
}
