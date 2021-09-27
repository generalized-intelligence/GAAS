/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: benchmark
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Code example of using the PSA GPU interface 
 *              in compiler time link with the appropiate pairwise algorithm.
 *				This example: (A) extract the candidates from a reference positions
 *								  or
 *							  (B) replicates and packs the candidates in an stream
 */

#include "../include/psa_commons.h"
#include "../include/psa_errors.h"
#include "../include/psa_pairwise.h"
#include "../include/psa_regions.h"
#include "../include/psa_sequences.h"
#include "../include/psa_time.h"

#include <stdio.h>
#include <stdlib.h>
#include <omp.h>

int main(int32_t argc, char *argv[])
{
	if(argc < 2) {
		printBuildInfo();
		printf("Usage: psa-benchmark regionsfile.regions \n");
		printf("Usage: or \n");
		printf("Usage: psa-benchmark regionsfile.regions genome.fasta\n");
		printf("example: bin/psa-benchmark input/regions.1M.prof.1000.400.12.regions \n");
		exit(E_PROGRAM_PARAMETERS);
	}

	regions_t	*regions 	= NULL;
    sequences_t *genome;
    const uint32_t numGenomes = 1;
    float workPerCandidate;
    char resultsFile[512];
    char *regionsFile = argv[1], *genomeFile;
	FILE *fp = NULL;

    /* Measure definitions */
    double ts, ts1, segTime;
    int32_t iter = 1, n;

	PSA_ERROR(printBuildInfo());

	fp = fopen(regionsFile, "rb");
	if (fp == NULL) return(E_OPENING_FILE);
	PSA_ERROR(initRegions(&regions, 0, 0, 0));
	PSA_ERROR(loadRegions(fp, regions));
	fclose(fp);

	if(regions->candidates->formats & SEQ_REF_POS){
		genomeFile = argv[2];
		fp = fopen(genomeFile, "rb");
		if (fp == NULL) return(E_OPENING_FILE);
		PSA_ERROR(initSequences(&genome, numGenomes));
		PSA_ERROR(loadSequenceMFASTA(fp, &genome->h_size, &genome->h_ASCII, &genome->h_ASCIIposition, &genome->formats));
		fclose(fp);
	}

	/* DEBUG REGIONS*/
	//PSA_ERROR(printRegions(regions, 4, 1));

	#ifdef REFERENCE
		PSA_ERROR(transformDataReferences(genome));
	#endif

	PSA_ERROR(transformDataQueries(regions->queries));
	PSA_ERROR(transformDataCandidates(regions->candidates));

	#if defined(CUDA) && defined(REFERENCE)
		PSA_ERROR(transferCPUtoGPUReference(genome, regions->queries, regions->candidates, regions->alignments));
	#endif
	#if defined(CUDA) && !defined(REFERENCE)
		PSA_ERROR(transferCPUtoGPUStream(regions->queries, regions->candidates, regions->alignments));
	#endif

	ts = sampleTime();

		#ifdef REFERENCE
			for(n = 0; n < iter; ++n)
			processPairwiseReference(genome, regions->candidates, regions->queries, regions->alignments);
		#else
			for(n = 0; n < iter; ++n)
			processPairwiseStream(regions->candidates, regions->queries, regions->alignments);
		#endif

	ts1 = sampleTime();
	workPerCandidate = regions->averageSizeQuery * (regions->distance * regions->averageSizeQuery);
	segTime = (ts1 - ts) / iter;
	printf("TIME: \t %f \t GCUPS: \t %f \n", segTime, (workPerCandidate * regions->candidates->num) / 1000000000 / segTime);
	printf("AVERAGE_QUERY_SIZE: \t %d, \t AVERAGE_CANDIDATES_PER_QUERY: \t %d, \t NUM_CANDIDATES: \t %d \n", 
			regions->averageSizeQuery, regions->averageCandidatesPerQuery, regions->candidates->num);
 
	#ifdef CUDA
		PSA_ERROR(transferGPUtoCPU(regions->alignments));
	#endif


	sprintf(resultsFile, "%s.%s", regionsFile, buildTag());
	fp = fopen(resultsFile, "wb");
	    if (fp == NULL) return(E_OPENING_FILE);
    PSA_ERROR(saveASCIIAlignments(fp, regions->alignments));
    fclose(fp);

    PSA_ERROR(freeRegions(&regions));

    return (SUCCESS);
}
