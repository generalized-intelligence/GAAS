/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_swwozniak_gpu
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Host functions for the SW-Gotoh GPU implementation
 *				using a wozniak approach. Paralelizing by anti-diagonals.
 */

#include "../../../include/psa_pairwise.h"
#include "../../../include/psa_pairwise_gpu.h"
#include <cuda_runtime.h>
#include <cuda.h>

#define WORD_SIZE 				32

#define MATCH_SCORE				 2
#define MISMATCH_SCORE 			-5
#define OPEN_INDEL_SCORE		-2
#define EXTEND_INDEL_SCORE		-1

#define MAX3(a,b,c)		((a >= b) ? ((a >= c) ? a : c) : ((b >= c) ? b : c))

#define BUILD_ALGORITHM	"SW-GOTOH WOZNIAK (LOCAL ALIGNMENT)"
#define BUILD_VERSION 	"GPU VERSION 32bits per cell"
#define BUILD_TAG		"wozniak.32b.tc.gpu"

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
	printf("CONFIG - MATCH: %d, MISMATCH: %d, INDEL_OPEN: %d, INDEL_EXTEND: %d\n",
			MATCH_SCORE, MISMATCH_SCORE, OPEN_INDEL_SCORE, EXTEND_INDEL_SCORE);
	printf("Build: %s - %s \n", __DATE__, __TIME__ );

	return (SUCCESS);
}

psaError_t transformDataQueries(sequences_t *queries)
{
	uint32_t idBase;

	for (idBase = 0; idBase < queries->numASCIIEntries; ++idBase)
		queries->h_ASCII[idBase] = ASCIItoUpperCase(queries->h_ASCII[idBase]);

	queries->formats |= SEQ_ASCII;
	
	return (SUCCESS);
}

psaError_t transformDataCandidates(sequences_t *candidates)
{
	uint32_t idBase;
	
	for (idBase = 0; idBase < candidates->numASCIIEntries; ++idBase)
		candidates->h_ASCII[idBase] = ASCIItoUpperCase(candidates->h_ASCII[idBase]);

	candidates->formats |= SEQ_ASCII;

	return (SUCCESS);
}

psaError_t transferCPUtoGPUStream(sequences_t *queries, sequences_t *candidates, alignments_t *alignments)
{
	//allocate & transfer Queries to GPU
	CUDA_ERROR(cudaMalloc((void**)&queries->d_ASCII, queries->numASCIIEntries * sizeof(ASCIIEntry_t)));
	CUDA_ERROR(cudaMemcpy(queries->d_ASCII, queries->h_ASCII, queries->numASCIIEntries * sizeof(ASCIIEntry_t), cudaMemcpyHostToDevice));

	CUDA_ERROR(cudaMalloc((void**)&queries->d_ASCIIposition, queries->num * sizeof(uint32_t)));
	CUDA_ERROR(cudaMemcpy(queries->d_ASCIIposition, queries->h_ASCIIposition, queries->num * sizeof(uint32_t), cudaMemcpyHostToDevice));


	//allocate & transfer FMIndex to GPU
	CUDA_ERROR(cudaMalloc((void**)&candidates->d_ASCII, candidates->numASCIIEntries * sizeof(ASCIIEntry_t)));
	CUDA_ERROR(cudaMemcpy(candidates->d_ASCII, candidates->h_ASCII, candidates->numASCIIEntries * sizeof(ASCIIEntry_t), cudaMemcpyHostToDevice));

	CUDA_ERROR(cudaMalloc((void**)&candidates->d_ASCIIposition, candidates->num * sizeof(uint32_t)));
	CUDA_ERROR(cudaMemcpy(candidates->d_ASCIIposition, candidates->h_ASCIIposition, candidates->num * sizeof(uint32_t), cudaMemcpyHostToDevice));


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

psaError_t processPairwiseStream(sequences_t *candidates, sequences_t *queries, alignments_t *alignments)
{
	return(localProcessPairwiseStream(candidates, queries, alignments));
}




