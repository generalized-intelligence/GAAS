/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_swfarrar_gpu
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Device functions for the SW-Gotoh GPU implementation
 *				using a farrar approach. Paralelizing by columns.
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

#define MAX3(a,b,c)				(MAX(MAX(a, b), c))

#define WARP_SIZE				32
#define MAX_THREADS_PER_SM		128
#define CUDA_NUM_THREADS		128

#define THREADS_PER_SEGMENT		32
#define NUM_SEGMENTS			(DIV_CEIL(QUERIES_SIZE, THREADS_PER_SEGMENT))
#define NUM_SW_PER_BLOCK		(MAX_THREADS_PER_SM / THREADS_PER_SEGMENT)
#define NUM_WARPS				(MAX_THREADS_PER_SM / WARP_SIZE)




//inline __device__ int32_t reduce_max(int32_t values, int32_t localThreadIdx)
inline __device__ int32_t reduce_max(int32_t values)
{
	//TODO: Remember the maximal ROW (reduce using KEY and VALUE)
	//const int32_t SEGMENTS_PER_WARP = 1;
	for (int i = 1; i < THREADS_PER_SEGMENT; i *= 2){
		int n = __shfl_down(values, i, 32);
			values = MAX(n, values);
	}
	//values = __shfl(values, ((localThreadIdx % SEGMENTS_PER_WARP) * THREADS_PER_SEGMENT));
	return(values);
}

__global__
void localProcessSWFarrar(ASCIIEntry_t *d_CandidatesASCII, uint32_t *d_CandidatesASCIIposition,
		   ASCIIEntry_t *d_QueriesASCII, uint32_t *d_QueriesASCIIposition,
		   alignmentInfo_t *d_AlignmentsInfo, alignmentEntry_t *d_AlignmentsResults,
		   uint32_t querySize, uint32_t candidateSize, uint32_t candidatesNum)
{
	const uint32_t idThread 			= blockIdx.x * MAX_THREADS_PER_SM + threadIdx.x;
	const uint32_t idCandidate 			= idThread / THREADS_PER_SEGMENT;
	const uint32_t idThreadLocalSegment = idThread % THREADS_PER_SEGMENT;

	if ((threadIdx.x < MAX_THREADS_PER_SM) && (idCandidate < candidatesNum))
	{
		const char* candidate 	= d_CandidatesASCII + d_CandidatesASCIIposition[idCandidate];
		const char* query 		= d_QueriesASCII + d_QueriesASCIIposition[d_AlignmentsInfo[idCandidate]];

		const uint32_t numSegments = DIV_CEIL(querySize, THREADS_PER_SEGMENT);

		int32_t vMaxScore 	= 0; 	// Trace the highest score of the whole SW matrix (using a Segment)
		int32_t maxScore    = 0;    // Final maximal score (Scalar value)
		int32_t maxColumn   = 0;		    // Final maximal score (Scalar value)
		//int32_t maxRow    = 0;    		// Final maximal score (Scalar value)
		int32_t tmpMax		= 0;

		__shared__  int32_t CTA_pvHStore [THREADS_PER_SEGMENT * NUM_SEGMENTS * NUM_SW_PER_BLOCK];
		__shared__  int32_t CTA_pvHLoad  [THREADS_PER_SEGMENT * NUM_SEGMENTS * NUM_SW_PER_BLOCK];
		__shared__  int32_t CTA_pvE 	 [THREADS_PER_SEGMENT * NUM_SEGMENTS * NUM_SW_PER_BLOCK];

		int32_t *pvHLoad  = CTA_pvHLoad  + ((THREADS_PER_SEGMENT * NUM_SEGMENTS) * (idCandidate % NUM_SW_PER_BLOCK));
		int32_t *pvHStore = CTA_pvHStore + ((THREADS_PER_SEGMENT * NUM_SEGMENTS) * (idCandidate % NUM_SW_PER_BLOCK));
		int32_t *pvE	  = CTA_pvE 	 + ((THREADS_PER_SEGMENT * NUM_SEGMENTS) * (idCandidate % NUM_SW_PER_BLOCK));

		int32_t idColumn, idSegmentCell, idSegment, idColumnCell = idThreadLocalSegment;

		// All is initialized to zero because is local
		for (idSegment = 0; idSegment < numSegments; ++idSegment){
			pvHStore[idColumnCell] 	= 0;
			pvHLoad[idColumnCell] 	= 0;
			pvE[idColumnCell] 		= 0;
			idColumnCell += THREADS_PER_SEGMENT;
		}

		for (idColumn = 0; idColumn < CANDIDATES_SIZE; ++idColumn) {
			const char candidateBase = candidate[idColumn];
			int32_t vE, vF = 0, vW, idColumnCell = idThreadLocalSegment;
			//int32_t vRowMax = idThreadLocalSegment;
			int32_t *pv = pvHLoad;
			vMaxScore = 0;

			// Reads the previous and last vH column value
			int32_t vH = pvHStore[(NUM_SEGMENTS - 1) * THREADS_PER_SEGMENT + idThreadLocalSegment];

			// Shifts the last column value 1 element to the left.
			vH = __shfl_down(vH, 1); //ok
			vH  = (idThreadLocalSegment == THREADS_PER_SEGMENT-1) ? 0 : vH;

			// Swap the 2 H buffers
			pvHLoad = pvHStore;
			pvHStore = pv;

			// Inner loop to process each query segment
			for (idSegment = 0; idSegment < NUM_SEGMENTS; ++idSegment) {
				//TODO: discuss include query profile
				vW = (candidateBase == query[idColumnCell]) ? MATCH_SCORE : MISMATCH_SCORE;
				vW = (idColumnCell < querySize) ? vW : 0;

				//Adds to the vH of the previous column the W(q_i, d_j) value
				vH += vW;

				vE = pvE[idColumnCell];
				//Get max from vH, vF and vE
				vH = MAX3(vH, vF, vE);

				//Get max from vH, vE and vF
				vMaxScore = MAX(vMaxScore, vH);
				//vMaxRow  = assigned from vMaxScore o vH

				//Save vH values
				pvHStore[idColumnCell] = vH;

				// {Calculate new vE_ij value}
				vH += OPEN_INDEL_SCORE;
				vE += EXTEND_INDEL_SCORE;
				pvE[idColumnCell] = MAX(vE, vH);

				//{Calculate new vF_ij value}
				vF += EXTEND_INDEL_SCORE;
				vF = MAX(vF, vH);

				// Load the next vH value to process
				vH = pvHLoad[idColumnCell];
				idColumnCell += THREADS_PER_SEGMENT;
			}

			// Lazy_F loop: disallow adjacent insertion and deletion don't update E(i, j)
			// Fix all the F dependences contributions (evaluates all the combinations in parallel)
			for (idSegmentCell = 0; idSegmentCell < THREADS_PER_SEGMENT; ++idSegmentCell) {
				vF = __shfl_down(vF, 1);
				vF  = (idThreadLocalSegment == THREADS_PER_SEGMENT-1) ? 0 : vF;
				idColumnCell = idThreadLocalSegment;
				for (idSegment = 0; idSegment < NUM_SEGMENTS; ++idSegment) {
					vH = MAX(pvHStore[idColumnCell], vF);
					pvHStore[idColumnCell] = vH;
					vH += OPEN_INDEL_SCORE;
					vF += EXTEND_INDEL_SCORE;
					if (__all(vF <= vH))
						goto exit_lazy_f_loop;
					//vF += EXTEND_INDEL_SCORE;
					idColumnCell += THREADS_PER_SEGMENT;
				}
			}

		exit_lazy_f_loop:
			// Record the max score of current column
			tmpMax = reduce_max(vMaxScore);
			maxScore  = ((idThreadLocalSegment == 0) && (tmpMax >= maxScore)) ? tmpMax   : maxScore;
			maxColumn = ((idThreadLocalSegment == 0) && (tmpMax >= maxScore)) ? idColumn : maxColumn;
		}

		if(idThreadLocalSegment == 0){
			d_AlignmentsResults[idCandidate].score  = maxScore;
			d_AlignmentsResults[idCandidate].column = 0;
			//d_AlignmentsResults[idCandidate].column = maxColumn;
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
	localProcessSWFarrar<<<blocks, threads>>>(candidates->d_ASCII, candidates->d_ASCIIposition,
								 			queries->d_ASCII, queries->d_ASCIIposition, 
											alignments->d_info, alignments->d_results,
								 			querySize, candidateSize, candidates->num);

	cudaThreadSynchronize();

	return (SUCCESS);
}
