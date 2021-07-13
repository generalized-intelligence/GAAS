/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: genRegions
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Generate candidates from *.profile
 */

#include "../include/psa_sequences.h"
#include "../include/psa_regions.h"
#include "../include/psa_profile.h"
#include "../include/psa_alignments.h"

psaError_t gatherCandidatesASCIIStream(regions_t *regions, uint32_t *h_genomePositionCandidates, sequences_t *genome)
{
	sequences_t *queries = regions->queries;
	sequences_t *candidates = regions->candidates;
	alignments_t *alignments = regions->alignments;

	uint32_t idRecoveredSequence, sizeRecoveredSequence, startPositionSequence, idSequenceBase, recoveredBases = 0;
	const uint32_t numCandidates = candidates->num;

	for(idRecoveredSequence = 0; idRecoveredSequence < numCandidates; ++idRecoveredSequence){
		sizeRecoveredSequence = regions->distance * queries->h_size[alignments->h_info[idRecoveredSequence]];
		candidates->h_ASCIIposition[idRecoveredSequence] = recoveredBases;
		candidates->h_size[idRecoveredSequence] = sizeRecoveredSequence;
		startPositionSequence = h_genomePositionCandidates[idRecoveredSequence];

		for(idSequenceBase = 0; idSequenceBase < sizeRecoveredSequence; ++idSequenceBase)
			candidates->h_ASCII[recoveredBases + idSequenceBase] = genome->h_ASCII[startPositionSequence + idSequenceBase];

		recoveredBases += sizeRecoveredSequence;

	}

	return (SUCCESS);
}

psaError_t gatherCandidatesASCIIReference(regions_t *regions, sequences_t* genome)
{
	sequences_t *queries = regions->queries;
	sequences_t *candidates = regions->candidates;
	alignments_t *alignments = regions->alignments;

	uint32_t idRecoveredSequence, sizeRecoveredSequence, startPositionSequence, idSequenceBase, recoveredBases = 0;
	const uint32_t numCandidates = candidates->num;

	for(idRecoveredSequence = 0; idRecoveredSequence < numCandidates; ++idRecoveredSequence){
		sizeRecoveredSequence = regions->distance * queries->h_size[alignments->h_info[idRecoveredSequence]];
		candidates->h_ASCIIposition[idRecoveredSequence] = recoveredBases;
		candidates->h_size[idRecoveredSequence] = sizeRecoveredSequence;
		startPositionSequence = candidates->h_refPosition[idRecoveredSequence];

		for(idSequenceBase = 0; idSequenceBase < sizeRecoveredSequence; ++idSequenceBase)
			candidates->h_ASCII[recoveredBases + idSequenceBase] = genome->h_ASCII[startPositionSequence + idSequenceBase];

		recoveredBases += sizeRecoveredSequence;
	}

	return (SUCCESS);
}


int main(int argc, char *argv[])
{

	if(argc != 5){
		printf("Usage: gregions mode distance regionsfile.prof \n");
		printf("\t mode = 0 -> ASCII candidates \n");
		printf("\t mode = 1 -> Reference Candidates \n");
		printf("example: bin/gregions 1 1.2 input/regions-score.1M.prof input/human_genome.fasta \n");
		exit(0);
	}

    regions_t *regions;
    sequences_t *genome;
    seq_format_t candidatesMode = SEQ_ASCII, queriesMode = SEQ_ASCII;
	uint32_t mode = atof(argv[1]);
    float distance = atof(argv[2]);
    char *profileFile = argv[3], *genomeFile = argv[4];
    char regionsFile[512];
	FILE *fp = NULL;

    const uint32_t numGenomes = 1;
    uint32_t *h_genomePositionCandidates = NULL, *h_regionsProfiledScores = NULL;
	uint32_t numQueries = 0, totalSizeQueries = 0, numCandidates = 0, totalSizeCandidates = 0;

	if(mode == 1) candidatesMode |= SEQ_REF_POS;

	fp = fopen(profileFile, "rb");
	if (fp == NULL) return(E_OPENING_FILE);
	PSA_ERROR(countRegionsProfile(fp, distance, &numQueries, &totalSizeQueries, &numCandidates, &totalSizeCandidates));
	PSA_ERROR(initRegions(&regions, numQueries, numCandidates, numCandidates));
	PSA_ERROR(allocateRegions(regions, distance, totalSizeQueries, queriesMode, totalSizeCandidates, candidatesMode));

	h_genomePositionCandidates = (uint32_t *) malloc(numCandidates * sizeof(uint32_t));
	if (h_genomePositionCandidates == NULL) PSA_ERROR(E_ALLOCATE_MEM);
	h_regionsProfiledScores = (uint32_t *) malloc(numCandidates * sizeof(uint32_t));
	if (h_regionsProfiledScores == NULL) PSA_ERROR(E_ALLOCATE_MEM);
	
	rewind(fp);
	if(mode == 1) PSA_ERROR(loadRegionsProfile(fp, regions->queries->h_ASCII, regions->queries->h_size, regions->queries->h_ASCIIposition,
		    				regions->candidates->h_refPosition, regions->alignments->h_info, h_regionsProfiledScores));
	else PSA_ERROR(loadRegionsProfile(fp, regions->queries->h_ASCII, regions->queries->h_size, regions->queries->h_ASCIIposition,
				   h_genomePositionCandidates, regions->alignments->h_info, h_regionsProfiledScores));
	regions->queries->formats |= queriesMode;
	fclose(fp);

	fp = fopen(genomeFile, "rb");
	if (fp == NULL) return(E_OPENING_FILE);
	PSA_ERROR(initSequences(&genome, numGenomes));
	PSA_ERROR(loadSequenceMFASTA(fp, &genome->h_size, &genome->h_ASCII, &genome->h_ASCIIposition, &genome->formats));
    fclose(fp);

    /* recover the candidates from genome in ASCII rep */
    if (mode == 1) PSA_ERROR(gatherCandidatesASCIIReference(regions, genome));
    else PSA_ERROR(gatherCandidatesASCIIStream(regions, h_genomePositionCandidates, genome));
	regions->candidates->formats |= candidatesMode;

	/* DEBUG REGIONS*/
	#if defined(DEBUG)
		PSA_ERROR(printRegions(regions, 4, 1));
	#endif

	if (mode == 1) sprintf(regionsFile, "%s.%u.%u.%u.ref.regions", profileFile, regions->queries->num, regions->averageSizeQuery, regions->averageCandidatesPerQuery);\
	else sprintf(regionsFile, "%s.%u.%u.%u.regions", profileFile, regions->queries->num, regions->averageSizeQuery, regions->averageCandidatesPerQuery);

    fp = fopen(regionsFile, "wb");
	if (fp == NULL) return(E_OPENING_FILE);
	PSA_ERROR(saveRegions(fp, regions));
    fclose(fp);

	PSA_ERROR(freeRegions(&regions));
	free(h_genomePositionCandidates);
	free(h_regionsProfiledScores);

    return (SUCCESS);
}

