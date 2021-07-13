/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_swgotoh_registers_16b_gpu
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 *			  Jacopo Pantaleoni <jpantaleoni@nvidia.com>
 * DESCRIPTION: Device functions for the SW-Gotoh GPU implementation:
 *				Using a 16 bits of representation per cell and intermediate column. 
 */

extern "C" {
#include "../../../include/psa_pairwise_gpu.h"
}
#include <cuda_runtime.h>
#include <cuda.h>

#define MATCH_SCORE				((short) 2)
#define MISMATCH_SCORE 			((short)-5)
#define OPEN_INDEL_SCORE		((short)-2)
#define EXTEND_INDEL_SCORE		((short)-1)

#define V4_PACKET1(NUM_A,NUM_B,NUM_C,NUM_D)     ((NUM_A << 24) | (NUM_B << 16) | (NUM_C << 8) | NUM_D)
#define V4_PACKET4(NUM) 						((NUM   << 24) | (NUM   << 16) | (NUM   << 8) | NUM)

#ifndef QUERIES_SIZE
	#define QUERIES_SIZE 		100
#endif

#ifndef CANDIDATES_SIZE
	#define CANDIDATES_SIZE 	120
#endif

#define MAX3(a,b,c)				(MAX(MAX(a, b), c))

#define WARP_SIZE				32
#define MAX_THREADS_PER_SM		128
#define CUDA_NUM_THREADS		128

#define THREADS_PER_SEGMENT		32
#define NUM_SW_PER_BLOCK		(MAX_THREADS_PER_SM / THREADS_PER_SEGMENT)
#define NUM_WARPS				(MAX_THREADS_PER_SM / WARP_SIZE)

#define BAND_LEN			 	8
#define MAX_QUERY_SIZE			200

inline __device__
int32_t max3(const int32_t op1, const int32_t op2, const int32_t op3)
{
    uint32_t r;
    asm( "  vmax.s32.s32.s32.max %0, %1, %2, %3;"               : "=r"(r)                              : "r"(op1), "r"(op2), "r"(op3) );
    return r;
}


inline __device__
void update_band(int32_t idRow, char q_i, char *ref_cache, int16_t *H_band, int16_t *F_band,
				short2 *temp, int16_t *H_maxScore)
{
	int16_t H_diag    = H_band[0];
	H_band[0] 		  = temp[idRow].x;
    int16_t E 	      = temp[idRow].y;

    #pragma unroll
    for (uint32_t j = 1; j <= BAND_LEN; ++j)
    {
        // update F
        const int16_t ftop = F_band[j] + EXTEND_INDEL_SCORE;
        const int16_t htop = H_band[j] + OPEN_INDEL_SCORE;
        F_band[j] = MAX(ftop, htop);

        // update E
        const int16_t eleft = E + EXTEND_INDEL_SCORE;
        const int16_t hleft = H_band[j-1] + OPEN_INDEL_SCORE;
        E = MAX(eleft, hleft);

        const    char r_j      = ref_cache[j-1];
        const int16_t W_ij     = (r_j == q_i) ? MATCH_SCORE : MISMATCH_SCORE;
        const int16_t diagonal = H_diag + W_ij;
        const int16_t top      = F_band[j];
        const int16_t left     = E;
        	  int16_t hi       = MAX3(left, top, diagonal);
                      hi       = MAX(hi, (short) 0);
        H_diag                 = H_band[j];
        H_band[j]              = hi;
        (*H_maxScore) = MAX((*H_maxScore), hi);
    }

    // save the last entry of the band
    temp[idRow] = make_short2(H_band[BAND_LEN], E);
    //(* H_maxScore) = MAX((* H_maxScore), H_band[BAND_LEN]);
}

__global__
void localProcessSWTiling(ASCIIEntry_t *d_CandidatesASCII, uint32_t *d_CandidatesASCIIposition,
		   ASCIIEntry_t *d_QueriesASCII, uint32_t *d_QueriesASCIIposition,
		   alignmentInfo_t *d_AlignmentsInfo, alignmentEntry_t *d_AlignmentsResults,
		   uint32_t querySize, uint32_t candidateSize, uint32_t candidatesNum)
{
	const uint32_t idCandidate  = blockIdx.x * MAX_THREADS_PER_SM + threadIdx.x;

	if (idCandidate < candidatesNum)
	{
		const char* candidate 	= d_CandidatesASCII + d_CandidatesASCIIposition[idCandidate];
		const char* query 		= d_QueriesASCII + d_QueriesASCIIposition[d_AlignmentsInfo[idCandidate]];

		short2  temp[MAX_QUERY_SIZE];
		char    r_cache[BAND_LEN];
		int16_t H_band[BAND_LEN + 1];
		int16_t F_band[BAND_LEN + 1];
		char q_i;

		const int32_t numRows = querySize, numColumns = candidateSize;
		int32_t idColumn, idRow, idBand;
		int16_t H_maxScore = 0;

		for(idBand = 0; idBand < MAX_QUERY_SIZE; ++idBand){
			temp[idBand].x = 0;
			temp[idBand].y = 0;
		}

		// Compute Score SW-GOTOH
		for(idColumn = 0; idColumn < numColumns; idColumn += BAND_LEN){

	        // load a block of entries from the reference
	        #pragma unroll
	        for (uint32_t idBand = 0; idBand < BAND_LEN; ++idBand)
	            r_cache[idBand] = candidate[idColumn + idBand];

	        // initialize the first band
	        #pragma unroll
	        for (uint32_t idBand = 0; idBand <= BAND_LEN; ++idBand){
	            H_band[idBand] = 0;
	            F_band[idBand] = 0;
	        }

	        for(idRow = 0; idRow < numRows; ++idRow){
	        	q_i = query[idRow];
				update_band(idRow, q_i, r_cache, H_band, F_band, temp, &H_maxScore);
	        }
		}

		d_AlignmentsResults[idCandidate].score  = H_maxScore;
		d_AlignmentsResults[idCandidate].column = 0;
		//d_AlignmentsResults[idCandidate].column = maxColumn;
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

	printf("Grid Size: %d, Block Size: %d, Total alignments: %d\n", blocks, threads, candidates->num);
	localProcessSWTiling<<<blocks, threads>>>(candidates->d_ASCII, candidates->d_ASCIIposition,
								 			queries->d_ASCII, queries->d_ASCIIposition, 
											alignments->d_info, alignments->d_results,
								 			querySize, candidateSize, candidates->num);

	cudaThreadSynchronize();

	return (SUCCESS);
}
