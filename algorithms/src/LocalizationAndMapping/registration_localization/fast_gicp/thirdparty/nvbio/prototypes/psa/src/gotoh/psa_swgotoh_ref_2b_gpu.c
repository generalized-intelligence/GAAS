/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_swgotoh_ref_2b_gpu
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Host functions for the SW-Gotoh GPU implementation using:
 *				(A) pack 4 SW in the same register using native video instructions.
 *				(B) 32bit video resources + and store the temporal columns with 8 bits.
 *				(C) bases are represented using 2 bits/base.
 *				(D) reads the candidates from the reference (check the impact of random accesses)
 */

#include "../../include/psa_pairwise.h"
#include "../../include/psa_pairwise_gpu.h"
#include <cuda_runtime.h>
#include <cuda.h>

#define WORD_SIZE 			32

#define MATCH_SCORE			 2
#define MISMATCH_SCORE 		-5
#define OPEN_INDEL_SCORE	-2
#define EXTEND_INDEL_SCORE	-1

#define MAX3(a,b,c)		((a >= b) ? ((a >= c) ? a : c) : ((b >= c) ? b : c))

#define BUILD_ALGORITHM	"SW-GOTOH: VIDEO INST USING REFERENCE POS (LOCAL ALIGNMENT)"
#define BUILD_VERSION 	"GPU VERSION 8bits per cell"
#define BUILD_TAG		"gotoh.video.ref.8b.gpu"

#define CUDA_ERROR(error) (HandleError(error, __FILE__, __LINE__ ))

static void HandleError( cudaError_t err, const char *file,  int32_t line ) {
   	if (err != cudaSuccess) {
      		printf( "%s in %s at line %d\n", cudaGetErrorString(err),  file, line );
       		exit( EXIT_FAILURE );
   	}
}

char* buildTag()
{
	return (BUILD_TAG);
}

psaError_t printBuildInfo()
{
	printf("%s \n", BUILD_ALGORITHM);
	printf("%s \n", BUILD_VERSION);
	printf("CONFIG - MATCH: %d, MISMATCH: %d, INDEL_OPEN: %d, INDEL_EXTEND %d\n",
			MATCH_SCORE, MISMATCH_SCORE, OPEN_INDEL_SCORE, EXTEND_INDEL_SCORE);
	printf("Build: %s - %s \n", __DATE__, __TIME__ );

	return (SUCCESS);
}

psaError_t transformDataReferences(sequences_t *references)
{
	uint32_t idReference, position, size;
	uint32_t currentHlfRAWEntry = 0, numHlfRAWEntries;
	const uint32_t RAW_BASES_PER_ENTRY = UINT32_LENGTH / RAW_4B_LENGTH;

	references->numRAWHlfEntries = sequences_totalEntriesHlfRAW(references->h_size, references->num, RAW_4B_LENGTH);

	references->h_HlfRAW = (RAWHlfEntry_t *) malloc(references->numRAWHlfEntries * sizeof(RAWHlfEntry_t));
	if (references->h_HlfRAW == NULL) return (E_ALLOCATE_MEM);

	references->h_HlfRAWposition = (uint32_t *) malloc(references->num * sizeof(uint32_t));
	if (references->h_HlfRAWposition == NULL) return (E_ALLOCATE_MEM);

	for (idReference = 0; idReference < references->num; ++idReference)
	{
		position = references->h_ASCIIposition[idReference];
		size = references->h_size[idReference];
		numHlfRAWEntries = DIV_CEIL(size, RAW_BASES_PER_ENTRY);

		sequenceASCIItoRAW_2x32bits(references->h_ASCII + position, size, references->h_HlfRAW + currentHlfRAWEntry);
		references->h_HlfRAWposition[idReference] = currentHlfRAWEntry;
		currentHlfRAWEntry += numHlfRAWEntries;
	}

	references->formats |= SEQ_HLF_RAW;
	return (SUCCESS);
}

psaError_t transformDataQueries(sequences_t *queries)
{
	uint32_t idQuery, position, size;
	uint32_t currentHlfRAWEntry = 0, numHlfRAWEntries;
	const uint32_t RAW_BASES_PER_ENTRY = UINT32_LENGTH / RAW_4B_LENGTH;

	queries->numRAWHlfEntries = sequences_totalEntriesHlfRAW(queries->h_size, queries->num, RAW_4B_LENGTH);

	queries->h_HlfRAW = (RAWHlfEntry_t *) malloc(queries->numRAWHlfEntries * sizeof(RAWHlfEntry_t));
	if (queries->h_HlfRAW == NULL) return (E_ALLOCATE_MEM);

	queries->h_HlfRAWposition = (uint32_t *) malloc(queries->num * sizeof(uint32_t));
	if (queries->h_HlfRAWposition == NULL) return (E_ALLOCATE_MEM);

	for (idQuery = 0; idQuery < queries->num; ++idQuery)
	{
		position = queries->h_ASCIIposition[idQuery];
		size = queries->h_size[idQuery];
		numHlfRAWEntries = DIV_CEIL(size, RAW_BASES_PER_ENTRY);

		sequenceASCIItoRAW_2x32bits(queries->h_ASCII + position, size, queries->h_HlfRAW + currentHlfRAWEntry);
		queries->h_HlfRAWposition[idQuery] = currentHlfRAWEntry;
		currentHlfRAWEntry += numHlfRAWEntries;
	}

	queries->formats |= SEQ_HLF_RAW;
	return (SUCCESS);
}

psaError_t transformDataCandidates(sequences_t *candidates)
{
	uint32_t idCandidate, position, size;
	uint32_t currentHlfRAWEntry = 0, numHlfRAWEntries;
	const uint32_t RAW_BASES_PER_ENTRY = UINT32_LENGTH / RAW_4B_LENGTH;

	candidates->numRAWHlfEntries = sequences_totalEntriesHlfRAW(candidates->h_size, candidates->num, RAW_4B_LENGTH);

	candidates->h_HlfRAW = (RAWHlfEntry_t *) malloc(candidates->numRAWHlfEntries * sizeof(RAWHlfEntry_t));
	if (candidates->h_HlfRAW == NULL) return (E_ALLOCATE_MEM);

	candidates->h_HlfRAWposition = (uint32_t *) malloc(candidates->num * sizeof(uint32_t));
	if (candidates->h_HlfRAWposition == NULL) return (E_ALLOCATE_MEM);

	for (idCandidate = 0; idCandidate < candidates->num; ++idCandidate)
	{
		position = candidates->h_ASCIIposition[idCandidate];
		size = candidates->h_size[idCandidate];
		numHlfRAWEntries = DIV_CEIL(size, RAW_BASES_PER_ENTRY);

		sequenceASCIItoRAW_2x32bits(candidates->h_ASCII + position, size, candidates->h_HlfRAW + currentHlfRAWEntry);
		candidates->h_HlfRAWposition[idCandidate] = currentHlfRAWEntry;
		currentHlfRAWEntry += numHlfRAWEntries;
	}

	candidates->formats |= SEQ_HLF_RAW;
	return (SUCCESS);
}

psaError_t transferCPUtoGPUReference(sequences_t *references, sequences_t *queries, sequences_t *candidates, alignments_t *alignments)
{
	//allocate & transfer References to GPU
	CUDA_ERROR(cudaMalloc((void**)&references->d_HlfRAW, references->numRAWHlfEntries * sizeof(RAWHlfEntry_t)));
	CUDA_ERROR(cudaMemcpy(references->d_HlfRAW, references->h_HlfRAW, references->numRAWHlfEntries * sizeof(RAWHlfEntry_t), cudaMemcpyHostToDevice));

	//allocate & transfer Queries to GPU
	CUDA_ERROR(cudaMalloc((void**)&queries->d_HlfRAW, queries->numRAWHlfEntries * sizeof(RAWHlfEntry_t)));
	CUDA_ERROR(cudaMemcpy(queries->d_HlfRAW, queries->h_HlfRAW, queries->numRAWHlfEntries * sizeof(RAWHlfEntry_t), cudaMemcpyHostToDevice));

	CUDA_ERROR(cudaMalloc((void**)&queries->d_HlfRAWposition, queries->num * sizeof(uint32_t)));
	CUDA_ERROR(cudaMemcpy(queries->d_HlfRAWposition, queries->h_HlfRAWposition, queries->num * sizeof(uint32_t), cudaMemcpyHostToDevice));

	//allocate & transfer Candidates to GPU
	CUDA_ERROR(cudaMalloc((void**)&candidates->d_refPosition, candidates->num * sizeof(uint32_t)));
	CUDA_ERROR(cudaMemcpy(candidates->d_refPosition, candidates->h_refPosition, candidates->num * sizeof(uint32_t), cudaMemcpyHostToDevice));

	//allocate & initialize Results
	CUDA_ERROR(cudaMalloc((void**)&alignments->d_info, alignments->num * sizeof(alignmentInfo_t)));
	CUDA_ERROR(cudaMemcpy(alignments->d_info, alignments->h_info, alignments->num * sizeof(alignmentInfo_t), cudaMemcpyHostToDevice));

 	CUDA_ERROR(cudaMalloc((void**)&alignments->d_results, alignments->num * sizeof(alignmentEntry_t)));
 	CUDA_ERROR(cudaMemset(alignments->d_results, 0, alignments->num * sizeof(alignmentEntry_t)));

	return (SUCCESS);
}

psaError_t transferGPUtoCPU(alignments_t *alignments)
{	
	CUDA_ERROR(cudaMemcpy(alignments->h_results, alignments->d_results, alignments->num * sizeof(alignmentEntry_t), cudaMemcpyDeviceToHost));

	return (SUCCESS);
}

psaError_t processPairwiseReference(sequences_t* references, sequences_t *candidates, sequences_t *queries, alignments_t *alignments)
{
	return(localProcessPairwiseReference(references, candidates, queries, alignments));
}





