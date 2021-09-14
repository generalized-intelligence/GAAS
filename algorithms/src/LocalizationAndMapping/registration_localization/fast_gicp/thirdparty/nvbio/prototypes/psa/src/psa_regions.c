/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_regions
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface to process GEM regions.
 */

#include "../include/psa_regions.h"

psaError_t initRegions(regions_t **regions, uint32_t numQueries, uint32_t numCandidates, uint32_t numAlignments)
{
    (* regions) = (regions_t *) malloc(sizeof(regions_t));
    if((* regions) == NULL) 

    (* regions)->distance = 0.0;
    (* regions)->averageSizeQuery = 0;
    (* regions)->averageCandidatesPerQuery = 0;

    PSA_ERROR(initSequences(&((* regions)->candidates), numCandidates));
    PSA_ERROR(initSequences(&((* regions)->queries), numQueries));
    PSA_ERROR(initAlignments(&((* regions)->alignments), numAlignments));

    return (SUCCESS);
}

psaError_t allocateRegions(regions_t *regions, float distance, 
							  uint32_t totalSizeQueries, seq_format_t QUERIES_ASCII, 
							  uint32_t totalSizeCandidates, seq_format_t CANDIDATES_ASCII)
{
    if (regions == NULL) return (E_ALLOCATE_MEM);

    regions->distance = distance;
    regions->averageSizeQuery = totalSizeQueries / regions->queries->num;
    regions->averageCandidatesPerQuery = regions->candidates->num / regions->queries->num;

    PSA_ERROR(allocateSequences(regions->queries, totalSizeQueries, QUERIES_ASCII));
	PSA_ERROR(allocateSequences(regions->candidates, totalSizeCandidates, CANDIDATES_ASCII));
	PSA_ERROR(allocateAlignments(regions->alignments));

    return (SUCCESS);
}

psaError_t loadRegions(FILE *fp, regions_t *regions)
{
	uint32_t result;

    result = fread(&regions->distance, sizeof(float), 1, fp);
        if (result != 1) return(E_WRITING_FILE);
    result = fread(&regions->averageCandidatesPerQuery, sizeof(uint32_t), 1, fp);
        if (result != 1) return(E_WRITING_FILE);
    result = fread(&regions->averageSizeQuery, sizeof(uint32_t), 1, fp);
	    if (result != 1) return(E_WRITING_FILE);

	PSA_ERROR(loadSequences(fp, regions->queries));
	PSA_ERROR(loadSequences(fp, regions->candidates));
	PSA_ERROR(loadAlignments(fp, regions->alignments));

	return (SUCCESS);
}

psaError_t saveRegions(FILE *fp, regions_t *regions)
{
	uint32_t result;

    result = fwrite(&regions->distance, sizeof(float), 1, fp);
        if (result != 1) return(E_WRITING_FILE);
    result = fwrite(&regions->averageCandidatesPerQuery, sizeof(uint32_t), 1, fp);
        if (result != 1) return(E_WRITING_FILE);
    result = fwrite(&regions->averageSizeQuery, sizeof(uint32_t), 1, fp);
	    if (result != 1) return(E_WRITING_FILE);

	PSA_ERROR(saveSequences(fp, regions->queries));
	PSA_ERROR(saveSequences(fp, regions->candidates));
	PSA_ERROR(saveAlignments(fp, regions->alignments));

	return (SUCCESS);
}


psaError_t statistics(regions_t *regions)
{
	const uint32_t RANGE_SIZE = 10000;
	uint32_t idHistogram, idQuery;
	uint32_t queryHistogram[RANGE_SIZE];
	uint32_t numElements = RANGE_SIZE;

	uint32_t minQueryLong = regions->queries->h_size[0];
	uint32_t maxQueryLong = regions->queries->h_size[0];

	for(idHistogram = 0; idHistogram < numElements; idHistogram++)
		queryHistogram[idHistogram] = 0;

	for(idQuery = 0; idQuery < regions->queries->num; idQuery++){
		maxQueryLong = MAX(regions->queries->h_size[idQuery], maxQueryLong);
		minQueryLong = MIN(regions->queries->h_size[idQuery], minQueryLong);
	}

	for(idQuery = 0; idQuery < regions->queries->num; idQuery++)
		queryHistogram[regions->queries->h_size[idQuery]]++;

	printf("Number of queries: %d\n", regions->queries->num);
	printf("Number of candidates: %d\n", regions->candidates->num);
	printf("Ratio: %.2f\n", (float)regions->candidates->num/(float)regions->queries->num);
	printf("Minimum Query Longitud %d - Maximum Query Longitud %d\n", minQueryLong, maxQueryLong);

	for(idHistogram = 0; idHistogram < maxQueryLong; idHistogram++){
		printf("%d\n",queryHistogram[idHistogram]);
	}

    return (SUCCESS);
}

psaError_t printRegions(regions_t *regions, uint32_t numAlignments, uint32_t reformated)
{
	uint32_t i, idAlignment;
	char ASCIIBase;

	printf("REGIONS: \n");
	printf("distance: %f \n", regions->distance);
	printf("averageSizeQuery: %d \n", regions->averageSizeQuery);
	printf("averageCandidatesPerQuery: %d \n", regions->averageCandidatesPerQuery);

	printf("QUERIES: \n");
	printf("formats: %d \n", 	regions->queries->formats);
	printf("num: %d \n", 	regions->queries->num);
	printf("numASCIIEntries: %d \n", 	regions->queries->numASCIIEntries);

	for(idAlignment = 0;  idAlignment < numAlignments; ++idAlignment){
		printf("Queries %d: ", idAlignment);
		for(i = regions->queries->h_ASCIIposition[idAlignment]; i < regions->queries->h_ASCIIposition[idAlignment+1]; ++i){
			if(reformated) ASCIIBase = ASCIItoUpperCase(regions->queries->h_ASCII[i]);
			printf("%c", ASCIIBase);
		}
		printf("\n");
		printf("Query %d size: %d \n", idAlignment, regions->queries->h_size[0]);
	}

	printf("CANDIDATES: \n");
	printf("formats: %d \n", 	regions->candidates->formats);
	printf("num: %d \n", 	regions->candidates->num);
	printf("numASCIIEntries: %d \n", 	regions->candidates->numASCIIEntries);

	for(idAlignment = 0;  idAlignment < numAlignments; ++idAlignment){
		printf("Candidate %d: ", idAlignment);
		for(i = regions->candidates->h_ASCIIposition[idAlignment]; i < regions->candidates->h_ASCIIposition[idAlignment+1]; ++i){
			if(reformated) ASCIIBase = ASCIItoUpperCase(regions->candidates->h_ASCII[i]);
			printf("%c", ASCIIBase);
		}
		printf("\n");
		printf("Candidate %d size: %d \n", idAlignment, regions->candidates->h_size[0]);
	}

	return (SUCCESS);
}

psaError_t freeRegions(regions_t **regions)
{   
	if((* regions) != NULL){
		PSA_ERROR(freeSequences(&(* regions)->queries));
		PSA_ERROR(freeSequences(&(* regions)->candidates));
		PSA_ERROR(freeAlignments(&(* regions)->alignments));
	    
    	free((* regions));
    	(* regions) = NULL;
    }
	
	return (SUCCESS);
}
