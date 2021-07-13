/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_profile
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Process functions for GEM profile files.
 */

#include "../include/psa_profile.h"

psaError_t countCandidates(char* cadena, uint32_t *numberOfCandidates, uint32_t *querySize)
{
	int localNumCandidates = 0, i = 0, flagQueryMesured = 0;

	while(cadena[i] != '\n'){
		if(cadena[i] == '\t'){
			if(flagQueryMesured == 0){
				(* querySize) = i;
				flagQueryMesured = 1;
			}
			localNumCandidates++;
		}
		i++;
	}
	(* numberOfCandidates) = localNumCandidates;
    return (SUCCESS);
}

psaError_t countRegionsProfile(FILE *fp, float distance, 
								  uint32_t *retNumQueries, uint32_t *retTotalSizeQueries, 
								  uint32_t *retNumCandidates, uint32_t *retTotalSizeCandidates)
{   
	//Read all the file and return the number of candidates and queries
	char cadena[FILE_SIZE_LINES];
	uint32_t numCandidates = 0, numQueries = 0;
	uint32_t totalSizeCandidates = 0, totalSizeQueries = 0;
	uint32_t localCandidates = 0, currentQuerySize = 0;
	
	if (fgets(cadena, FILE_SIZE_LINES, fp) == NULL) return (E_READING_FILE);

	PSA_ERROR(countCandidates(cadena, &localCandidates, &currentQuerySize));
	numCandidates += localCandidates;
	totalSizeQueries += currentQuerySize;
	totalSizeCandidates += ((distance * currentQuerySize) * localCandidates);
	numQueries++;

	while((!feof(fp)) && (fgets(cadena, FILE_SIZE_LINES, fp) != NULL)){
		PSA_ERROR(countCandidates(cadena, &localCandidates, &currentQuerySize)); 

		numCandidates += localCandidates;
		totalSizeQueries += currentQuerySize;
		totalSizeCandidates += ((distance * currentQuerySize) * localCandidates);
		numQueries++;
	}

	(* retNumQueries) = numQueries;
	(* retNumCandidates) = numCandidates;
	(* retTotalSizeQueries) = totalSizeQueries;
	(* retTotalSizeCandidates) = totalSizeCandidates;

    return (SUCCESS);
}

psaError_t parseCandidate(char *candidate, uint32_t *retPosition, uint32_t *retScore)
{
	char area[50];
	char symbol[10];
	char position[15];
	char score[500];
	int tokenCount;

	tokenCount = sscanf(candidate,"%49[^:]:%9[^:]:%14[^:]:%s", area, symbol, position, score);
	if (tokenCount != 4) return (E_PARSING_REGPROF);

	(* retPosition) = (uint32_t) atoi(position);
	(* retScore) 	= (uint32_t) atoi(score);
		
    return (SUCCESS);
}

psaError_t processQuery(uint32_t queryNumber, char *textLine,
				 		   char *queries, uint32_t *h_PositionCandidates, uint32_t *h_AlignmentInfo, uint32_t *listResults,
				 		   uint32_t *retSizeQueries, uint32_t *retNumCandidates)
{
	uint32_t position,
		 	 numCandidates = 0,
		 	 sizeQuery,
		  	 result,
		 	 tokenCount;

	int32_t	 sizeLastCandidates,
		 	 sizeCurrentCandidate;

	char *pLastCandidates;

	char query[FILE_SIZE_LINES],
	  	 lastCandidates[FILE_SIZE_LINES],
	  	 currentCandidate[500];

	pLastCandidates = &lastCandidates[0];

  	sscanf(textLine, "%s\t%[^\n]", query, pLastCandidates);
	sizeQuery = strlen(query);
	sizeLastCandidates = strlen(pLastCandidates);
	memcpy(queries, query, sizeQuery);
	
	while (sizeLastCandidates > 0){
		tokenCount = sscanf(pLastCandidates,"%s", currentCandidate);
		if (tokenCount < 1) return (E_PARSING_REGPROF);

		PSA_ERROR(parseCandidate(currentCandidate, &position, &result));
		h_AlignmentInfo[numCandidates] = queryNumber;
		h_PositionCandidates[numCandidates] = position;
		listResults[numCandidates] = result;
		numCandidates++;

		//update values next iteration
		sizeCurrentCandidate = strlen(currentCandidate) + 1;
		pLastCandidates += sizeCurrentCandidate;
		sizeLastCandidates -= sizeCurrentCandidate;
	}

	//return the number of data used
	(* retSizeQueries) 	  = sizeQuery;
	(* retNumCandidates)  =	numCandidates;

    return (SUCCESS);
}

psaError_t loadRegionsProfile(FILE *fp, char *h_ASCIIQueries, uint32_t *h_ASCIISizeQueries, uint32_t *h_ASCIIPositionQueries,
								uint32_t *h_GenomePositionCandidates, uint32_t *h_AlignmentInfo, uint32_t *h_AlignmentScores)
{      
	char textLine[FILE_SIZE_LINES];
	uint32_t queryNumber = 0,
		 sizeCurrentQuery, 
		 numQueryCandidates,
		 processedCandidates = 0, 
		 processedBases = 0,
		 totalSizeQueries = 0;

	if (fgets(textLine, FILE_SIZE_LINES, fp) == NULL) 
		return (E_READING_FILE);

	PSA_ERROR(processQuery(queryNumber, textLine, 
				 h_ASCIIQueries, h_GenomePositionCandidates, h_AlignmentInfo, h_AlignmentScores, 
				 &sizeCurrentQuery, &numQueryCandidates));

	h_ASCIISizeQueries[queryNumber] = sizeCurrentQuery;
	h_ASCIIPositionQueries[queryNumber] = 0;
	processedBases = sizeCurrentQuery;
	processedCandidates += numQueryCandidates;
	queryNumber++;

	while((!feof(fp)) && (fgets(textLine, FILE_SIZE_LINES, fp) != NULL)){
		PSA_ERROR(processQuery(queryNumber, textLine,
					 h_ASCIIQueries + processedBases, 
					 h_GenomePositionCandidates + processedCandidates,
					 h_AlignmentInfo + processedCandidates,
					 h_AlignmentScores + processedCandidates, 
				     &sizeCurrentQuery, &numQueryCandidates));

		h_ASCIISizeQueries[queryNumber] = sizeCurrentQuery;
		h_ASCIIPositionQueries[queryNumber] = processedBases;
		processedBases += sizeCurrentQuery;
		processedCandidates += numQueryCandidates;
		queryNumber++;
	}

    return (SUCCESS);
}