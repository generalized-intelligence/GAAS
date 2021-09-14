/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_swgotoh_2b_mixed_gpu
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Device functions for the SW-Gotoh GPU implementation using:
 *				(A) pack 4 SW in the same register mixing integer instructions and video instructions.
 *				(B) 32bit integer resources + and store the temporal columns with 8 bits.
 *				(C) bases are represented using 2 bits/base.
 */

extern "C" {
#include "../../include/psa_pairwise_gpu.h"
}
#include <cuda_runtime.h>
#include <cuda.h>
#include "../../include/simd_functions.h"

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
#define VECTOR_ELEMENTS			4
#define MAX_QUERY_SIZE			QUERIES_SIZE
#define BIAS					16
#define RAW_BASES_PER_ENTRY		(UINT32_LENGTH / RAW_4B_LENGTH)

typedef union {
	uchar4 c4;
	uint32_t i;
}score_type;

typedef uchar4	score_type2;

inline __device__
uint32_t sim_vmaxu4(uint32_t NUM_A, uint32_t NUM_B){
	return(MAX(NUM_A & 0xFF000000, NUM_B & 0xFF000000) |
		   MAX(NUM_A & 0x00FF0000, NUM_B & 0x00FF0000) |
		   MAX(NUM_A & 0x0000FF00, NUM_B & 0x0000FF00) |
		   MAX(NUM_A & 0x000000FF, NUM_B & 0x000000FF));
}


inline __device__
void update_band(int32_t idRow, const uint32_t q_i, const uint32_t *ref_cache, uint32_t *H_band, uint32_t *F_band,
				uint32_t *H_temp, uint32_t *E_temp, uint32_t *H_maxScore,
				const uint32_t MATCH_SCORE, const uint32_t MISMATCH_SCORE,
				const uint32_t OPEN_INDEL_SCORE, const uint32_t EXTEND_INDEL_SCORE)
{

	//Biased Scores to 16
	const uint32_t   	ZERO		= 0x10101010;
	uint32_t			H_diag		= H_band[0];
						H_band[0]	= H_temp[idRow];
	uint32_t			E 	    	= E_temp[idRow];

    #pragma unroll
    for (uint32_t j = 1; j <= BAND_LEN; ++j)
    {
        // update F
        const uint32_t ftop 	   = F_band[j] - EXTEND_INDEL_SCORE;
        const uint32_t htop 	   = H_band[j] - OPEN_INDEL_SCORE;
        			   F_band[j]   = vmaxu4(ftop, htop);

        // update E
        const uint32_t eleft = E           - EXTEND_INDEL_SCORE;
        const uint32_t hleft = H_band[j-1] - OPEN_INDEL_SCORE;
        			   E     = vmaxu4(eleft, hleft);
        // update H
        const uint32_t   r_j      = ref_cache[j-1];
        const uint32_t   Eq       = vcmpeq4(r_j, q_i);
		const uint32_t   notEq    = ~Eq;
        const uint32_t   diagonal = (notEq & (H_diag - MISMATCH_SCORE)) | (Eq & (H_diag + MATCH_SCORE));

        const uint32_t   top      = F_band[j];
        const uint32_t   left     = E;
		      uint32_t   hi       = vmaxu4(vmaxu4(left, top), diagonal);
		     	  	     hi       = vmaxu4(hi, ZERO);

        H_diag                      = H_band[j];
        H_band[j]                   = hi;
        (* H_maxScore) = vmaxu4((* H_maxScore), hi);
    }

     H_temp[idRow] = H_band[BAND_LEN];
     E_temp[idRow] = E;
}

__global__
void localProcessSWTiling(RAWHlfEntry_t *d_CandidatesHlfRaw, uint32_t *d_CandidatesHlfRAWposition,
		   RAWHlfEntry_t *d_QueriesHlfRaw, uint32_t *d_QueriesHlfRAWposition,
		   alignmentInfo_t *d_AlignmentsInfo, alignmentEntry_t *d_AlignmentsResults,
		   uint32_t querySize, uint32_t candidateSize, uint32_t candidatesNum)
{
	const uint32_t idCandidate  = (blockIdx.x * MAX_THREADS_PER_SM + threadIdx.x) * VECTOR_ELEMENTS;

	if (idCandidate < (candidatesNum))
	{
		const RAWHlfEntry_t* candidate0 = d_CandidatesHlfRaw + d_CandidatesHlfRAWposition[idCandidate];
		const RAWHlfEntry_t* candidate1 = d_CandidatesHlfRaw + d_CandidatesHlfRAWposition[idCandidate + 1];
		const RAWHlfEntry_t* candidate2 = d_CandidatesHlfRaw + d_CandidatesHlfRAWposition[idCandidate + 2];
		const RAWHlfEntry_t* candidate3 = d_CandidatesHlfRaw + d_CandidatesHlfRAWposition[idCandidate + 3];

		const RAWHlfEntry_t* query0 	= d_QueriesHlfRaw + d_QueriesHlfRAWposition[d_AlignmentsInfo[idCandidate]];
		const RAWHlfEntry_t* query1 	= d_QueriesHlfRaw + d_QueriesHlfRAWposition[d_AlignmentsInfo[idCandidate + 1]];
		const RAWHlfEntry_t* query2 	= d_QueriesHlfRaw + d_QueriesHlfRAWposition[d_AlignmentsInfo[idCandidate + 2]];
		const RAWHlfEntry_t* query3 	= d_QueriesHlfRaw + d_QueriesHlfRAWposition[d_AlignmentsInfo[idCandidate + 3]];

		// All the scores have to be absolute numbers:
		// Original BWA Scores: Gap_Ex = -1; Gap_Op = -2; Match = 2; Miss = -5;
		const score_type   MATCH_SCORE			=	{ 2, 2, 2, 2};
		const score_type   MISMATCH_SCORE 		=	{ 5, 5, 5, 5};

		const score_type   OPEN_INDEL_SCORE		=	{ 2, 2, 2, 2};
		const score_type   EXTEND_INDEL_SCORE	=	{ 1, 1, 1, 1};

		//Biased Scores to 16
		const score_type   ZERO					=	{ BIAS, BIAS, BIAS, BIAS};

		score_type		r_cache[BAND_LEN];
		score_type	 	H_temp [MAX_QUERY_SIZE];
		score_type 		E_temp [MAX_QUERY_SIZE];
		score_type 		H_band [BAND_LEN + 1];
		score_type 		F_band [BAND_LEN + 1];

		const int32_t numRows = querySize, numColumns = candidateSize;
		int32_t idColumn, idRow, idBand;
		score_type H_maxScore = ZERO;

		uint32_t idCandidateEntry = 0;
		uint32_t entryCandidate0, entryCandidate1, entryCandidate2, entryCandidate3;

		for(idBand = 0; idBand < MAX_QUERY_SIZE; ++idBand){
			H_temp[idBand].i = ZERO.i;
			E_temp[idBand].i = ZERO.i;
		}

		// Compute Score SW-GOTOH
		for(idColumn = 0; idColumn < numColumns; idColumn += BAND_LEN){
			uint32_t idQueryEntry = 0;
        	uint32_t entryQuery0, entryQuery1, entryQuery2, entryQuery3;

			if((idColumn % RAW_BASES_PER_ENTRY) == 0){
				entryCandidate0 = candidate0[idCandidateEntry];
				entryCandidate1 = candidate1[idCandidateEntry];
				entryCandidate2 = candidate2[idCandidateEntry];
				entryCandidate3 = candidate3[idCandidateEntry];
				idCandidateEntry++;
			}

	        // Load a block of entries from the reference
	        #pragma unroll
	        for (uint32_t idBand = 0; idBand < BAND_LEN; ++idBand){
	            r_cache[idBand].c4 = make_uchar4(entryCandidate0 & 0x3,
	            							   entryCandidate1 & 0x3,
	            							   entryCandidate2 & 0x3,
	            							   entryCandidate3 & 0x3);
	        	entryCandidate0 >>= RAW_4B_LENGTH;
	        	entryCandidate1 >>= RAW_4B_LENGTH;
	        	entryCandidate2 >>= RAW_4B_LENGTH;
	        	entryCandidate3 >>= RAW_4B_LENGTH;
	        }

	        // Initialize the first band
	        #pragma unroll
	        for (uint32_t idBand = 0; idBand <= BAND_LEN; ++idBand){
	        	H_band[idBand].i = ZERO.i;
	        	F_band[idBand].i = ZERO.i;
	        }

			#pragma unroll 1
	        for(idRow = 0; idRow < numRows; ++idRow){

	        	entryQuery0 >>= RAW_4B_LENGTH;
	        	entryQuery1 >>= RAW_4B_LENGTH;
	        	entryQuery2 >>= RAW_4B_LENGTH;
	        	entryQuery3 >>= RAW_4B_LENGTH;

				if((idRow % RAW_BASES_PER_ENTRY) == 0){
					entryQuery0 = query0[idQueryEntry];
					entryQuery1 = query1[idQueryEntry];
					entryQuery2 = query2[idQueryEntry];
					entryQuery3 = query3[idQueryEntry];
					idQueryEntry++;
				}

	        	const score_type q_i = {entryQuery0 & 0x03, entryQuery1 & 0x03, entryQuery2 & 0x03, entryQuery3 & 0x03};
				update_band(idRow, q_i.i, (uint32_t *) r_cache, (uint32_t *) H_band, (uint32_t *) F_band, (uint32_t *) H_temp, (uint32_t *) E_temp,
						&H_maxScore.i, MATCH_SCORE.i, MISMATCH_SCORE.i, OPEN_INDEL_SCORE.i, EXTEND_INDEL_SCORE.i);
	        }
		}

		d_AlignmentsResults[idCandidate].score  	= H_maxScore.c4.x - BIAS;
		d_AlignmentsResults[idCandidate + 1].score  = H_maxScore.c4.y - BIAS;
		d_AlignmentsResults[idCandidate + 2].score  = H_maxScore.c4.z - BIAS;
		d_AlignmentsResults[idCandidate + 3].score  = H_maxScore.c4.w - BIAS;

		d_AlignmentsResults[idCandidate].column     = 0;
		d_AlignmentsResults[idCandidate + 1].column = 0;
		d_AlignmentsResults[idCandidate + 2].column = 0;
		d_AlignmentsResults[idCandidate + 3].column = 0;
	}
}


extern "C"
psaError_t localProcessPairwiseStream(sequences_t *candidates, sequences_t *queries, alignments_t *alignments)
{
	uint32_t blocks = DIV_CEIL(DIV_CEIL(candidates->num, VECTOR_ELEMENTS), CUDA_NUM_THREADS);
	uint32_t threads = CUDA_NUM_THREADS;
	uint32_t querySize = queries->h_size[0];
	uint32_t candidateSize = candidates->h_size[0];

	cudaThreadSetCacheConfig(cudaFuncCachePreferL1);

	printf("Grid Size: %d, Block Size: %d, Total alignments: %d, BAND_LEN: %d \n", blocks, threads, candidates->num, BAND_LEN);
	localProcessSWTiling<<<blocks, threads>>>(candidates->d_HlfRAW, candidates->d_HlfRAWposition,
								 			queries->d_HlfRAW, queries->d_HlfRAWposition,
											alignments->d_info, alignments->d_results,
								 			querySize, candidateSize, candidates->num - VECTOR_ELEMENTS);

	cudaThreadSynchronize();

	return (SUCCESS);
}
