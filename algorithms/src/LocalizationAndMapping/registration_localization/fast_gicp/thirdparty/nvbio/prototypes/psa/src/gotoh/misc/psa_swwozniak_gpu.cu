/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_swwozniak_gpu
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 *			  Jacopo Pantaleoni <jpantaleoni@nvidia.com>
 * DESCRIPTION: Device functions for the SW-Gotoh GPU implementation
 *				using a wozniak approach. Paralelizing by anti-diagonals.
 *				The implementation is warp-wide using shuffle instructions.
 */

extern "C" {
#include "../../../include/psa_pairwise_gpu.h"
}
#include <cuda_runtime.h>
#include <cuda.h>

#define MATCH_SCORE				 2
#define MISMATCH_SCORE 			-5
#define OPEN_INDEL_SCORE		-2
#define EXTEND_INDEL_SCORE		-1

#ifndef QUERIES_SIZE
	#define QUERIES_SIZE 		100
#endif

#ifndef CANDIDATES_SIZE
	#define CANDIDATES_SIZE 	120
#endif

#define MAX3(a,b,c)				((a >= b) ? ((a >= c) ? a : c) : ((b >= c) ? b : c))
#define MIN3(a,b,c)				((a <  b) ? ((a <  c) ? a : c) : ((b <  c) ? b : c))

#define WARP_SIZE				32
#define MAX_THREADS_PER_SM		128
#define CUDA_NUM_THREADS		128

#define THREADS_PER_SEGMENT		32
#define NUM_SEGMENTS			(DIV_CEIL(QUERIES_SIZE, THREADS_PER_SEGMENT))
#define NUM_SW_PER_BLOCK		(MAX_THREADS_PER_SM / THREADS_PER_SEGMENT)
#define NUM_WARPS				(MAX_THREADS_PER_SM / WARP_SIZE)




inline __device__ int32_t reduce_max(int32_t values, int32_t column)
{
	//TODO: Remember the maximal ROW (reduce using KEY and VALUE)
	for (int i = 1; i < THREADS_PER_SEGMENT; i *= 2){
		//TODO: Pack the 2 values to save 1 SHUFFLE & CMOV ops
		int newValue  = __shfl_down(values, i, 32);
		int newColumn = __shfl_down(column, i, 32);
		values = MAX(newValue, values);
		column = (newValue >= values) ? newColumn : column;
	}
	return(values);
}

__global__
void localProcessSWWozniak(ASCIIEntry_t *d_CandidatesASCII, uint32_t *d_CandidatesASCIIposition,
		   ASCIIEntry_t *d_QueriesASCII, uint32_t *d_QueriesASCIIposition,
		   alignmentInfo_t *d_AlignmentsInfo, alignmentEntry_t *d_AlignmentsResults,
		   uint32_t querySize, uint32_t candidateSize, uint32_t candidatesNum)
{
	const uint32_t idThread 			= blockIdx.x * MAX_THREADS_PER_SM + threadIdx.x;
	const uint32_t idCandidate 			= idThread / THREADS_PER_SEGMENT;
	const uint32_t idThreadLocalSegment = idThread % THREADS_PER_SEGMENT;

	if ((threadIdx.x < MAX_THREADS_PER_SM) && (idCandidate < candidatesNum)){

		typedef int32_t score_type;
		__shared__ short2 tmpHValues[CANDIDATES_SIZE * NUM_SW_PER_BLOCK];
		short2 *temp = tmpHValues + (CANDIDATES_SIZE * (idCandidate % NUM_WARPS));

		const char* candidate = d_CandidatesASCII + d_CandidatesASCIIposition[idCandidate];
		const char* query 	  = d_QueriesASCII + d_QueriesASCIIposition[d_AlignmentsInfo[idCandidate]];

		// local scores
		score_type h_top, h_left, h_diag, hi;
		score_type e_top, ei;
		score_type f_left, fi;
		score_type h_maxScore = 0;
		uint32_t maxColumn = 0;

		// current reference string character
		uint8_t r_j;

		// per-thread cache for temp values and reference string characters
		// each thread loads a different value; cache values are shuffled down the warp at each iteration
		score_type temp_cache_h, temp_cache_f;
		uint8_t reference_cache;

		// width of the current warp-block stripe of the DP matrix (always WARP_SIZE except for the last stripe)
		uint32_t warp_block_width;

		// compute warp-block horizontal coordinate in DP matrix for this thread
		const uint32_t wi = idThreadLocalSegment + 1;

		// initialize the leftmost matrix column
		for(uint32_t i = idThreadLocalSegment; i < CANDIDATES_SIZE; i += WARP_SIZE)
			temp[i] = make_short2(0,0);

		for(uint32_t warp_block = 0; warp_block < QUERIES_SIZE; warp_block += WARP_SIZE){

			// width of this block
			warp_block_width = (warp_block + WARP_SIZE >= QUERIES_SIZE ? QUERIES_SIZE % WARP_SIZE : WARP_SIZE);
			// compute the horizontal coordinate of the current thread in the DP matrix (including border column)
			const uint32_t i = wi + warp_block;

			// set top boundary values
			h_top = 0;
			e_top = 0;

			// initialize diagonal
			h_diag = 0;

			// load the query string character for the current thread
			const uint8_t s_i = (i <= QUERIES_SIZE ? query[i - 1] : 0);

			// initialize the best score for this stripe
			score_type max_score = 0;

			// loop over all DP anti-diagonals, excluding the border row/column
			for(uint32_t block_diag = 2; block_diag <= warp_block_width + CANDIDATES_SIZE; block_diag += WARP_SIZE){

				// reload caches every WARP_SIZE diagonals
				const uint32_t thread_j = (block_diag - 2) + idThreadLocalSegment;

				if (thread_j < CANDIDATES_SIZE){
					temp_cache_h = temp[thread_j].x;
					temp_cache_f = temp[thread_j].y;
					reference_cache = candidate[(block_diag - 2) + idThreadLocalSegment];
				} else {
					temp_cache_h = 0;
					temp_cache_f = 0;
					reference_cache = 0;
				}

				for(uint32_t diag = block_diag; diag < block_diag + WARP_SIZE; diag++){

					// compute the length of this anti-diagonal (excluding border row/column)
					const uint32_t diag_len = MIN3(diag - 1, WARP_SIZE, warp_block_width);
					// compute vertical coordinate of the current cell in the DP matrix (including border column)
					const uint32_t j = diag - wi;

					// is the current cell inside the DP matrix?
					if (wi <= diag_len && j <= CANDIDATES_SIZE){
						if (wi == 1){
							// load new temp and reference values
							r_j = reference_cache;
							// initialize cell to the left of the current cell
							h_left = temp_cache_h;
							f_left = temp_cache_f;
						}

						// compute the match/mismatch score
						const score_type S_ij = (r_j == s_i) ? MATCH_SCORE : MISMATCH_SCORE;

						ei = MAX(e_top + EXTEND_INDEL_SCORE,
								 h_top + OPEN_INDEL_SCORE);
						fi = MAX(f_left + EXTEND_INDEL_SCORE,
								 h_left + OPEN_INDEL_SCORE);
						hi = MAX3(h_diag + S_ij,
								  ei,
								  fi);

						// clamp score to zero
						hi = MAX(hi, 0);

						// save off the last column
						if (wi == WARP_SIZE){
							temp[j - 1] = make_short2( hi, fi );
							// keep track of the best score in this stripe
							max_score = MAX( max_score, hi );
						}

						// save the best score across the entire matrix for local scoring
						if (hi >= h_maxScore){
							h_maxScore = hi;
							maxColumn = j;
							//h_alignment = make_uint2(j, i);
						}

						// current left becomes diagonal for next iteration on this lane
						h_diag = h_left;

						// current value becomes h_top for next iteration on this lane
						h_top = hi;
						e_top = ei;
					}

					// move previous cell reference value across the warp
					r_j = __shfl_up(r_j, 1);
					// hi becomes h_left on the next lane
					h_left = __shfl_up(hi, 1);
					f_left = __shfl_up(fi, 1);

					// push temp_cache and reference_cache values down the warp
					temp_cache_h    = __shfl_down(temp_cache_h, 1);
					temp_cache_f    = __shfl_down(temp_cache_f, 1);
					reference_cache = __shfl_down(reference_cache, 1);
				}
			}
		}

		h_maxScore = reduce_max(h_maxScore, maxColumn);
		if(idThreadLocalSegment == 0){
			d_AlignmentsResults[idCandidate].score = h_maxScore;
			//d_AlignmentsResults[idCandidate].column = maxColumn;
			d_AlignmentsResults[idCandidate].column = 0; //DEBUG
		}
	}
}

extern "C"
psaError_t localProcessPairwiseStream(sequences_t *candidates, sequences_t *queries, alignments_t *alignments)
{
	uint32_t alignmentsPerBlock = DIV_CEIL(MAX_THREADS_PER_SM, THREADS_PER_SEGMENT);
	uint32_t blocks = DIV_CEIL(candidates->num, alignmentsPerBlock);
	uint32_t threads = CUDA_NUM_THREADS;
	uint32_t querySize = queries->h_size[0];
	uint32_t candidateSize = candidates->h_size[0];

	printf("Grid Size: %d, Block Size: %d, Alignments Per Block: %d, Total alignments: %d\n", blocks, threads, alignmentsPerBlock, candidates->num);
	localProcessSWWozniak<<<blocks, threads>>>(candidates->d_ASCII, candidates->d_ASCIIposition,
								 			queries->d_ASCII, queries->d_ASCIIposition, 
											alignments->d_info, alignments->d_results,
								 			querySize, candidateSize, candidates->num);
	cudaThreadSynchronize();

	return (SUCCESS);
}
