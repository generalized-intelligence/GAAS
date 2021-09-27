/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: checksum
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: This tool process the outputs of two alignment sets and 
 *              process the differences with stadistics 
 *              (ideal to analize the required precision of the data) 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define DIV_CEIL(NUMERATOR,DENOMINATOR) (((NUMERATOR)+((DENOMINATOR)-1))/(DENOMINATOR))
#define CATCH_ERROR(error) {{if (error) { fprintf(stderr, "%s\n", processError(error)); exit(EXIT_FAILURE); }}}
#ifndef MIN
    #define MIN(_a, _b) (((_a) < (_b)) ? (_a) : (_b))
#endif

#ifndef TRUE
    #define TRUE	1
#endif

#ifndef FALSE
    #define FALSE	0
#endif

#ifndef MATCH_SCORE
    #define MATCH_SCORE	2
#endif

typedef struct {
    uint32_t column;
    uint32_t score;
} resEntry_t;

typedef struct {
    uint32_t numResults; 
    resEntry_t* h_results;
    char *h_verified;
} res_t;

int loadResults(const char *fn, void **results)
{
    FILE *fp = NULL;
    res_t *res = (res_t *) malloc(sizeof(res_t));
    uint32_t idResult, column, score, idCandidate;
    size_t result;

    fp = fopen(fn, "r");
    if (fp == NULL) return (2);

    res->h_results = NULL;
    fscanf(fp, "%u", &res->numResults);
    //DEBUG
    res->numResults -= 4;

    res->h_results = (resEntry_t *) malloc(res->numResults * sizeof(resEntry_t));
        if (res->h_results == NULL) return (45);
    res->h_verified = (char *) malloc(res->numResults * sizeof(char));
        if (res->h_verified == NULL) return (45);

    for(idResult = 0; idResult < res->numResults; idResult++){
        fscanf(fp, "%u %u %u", &idCandidate, &score, &column);
        res->h_results[idResult].column = column;
        res->h_results[idResult].score = score;
        res->h_verified[idResult] = TRUE;
    }

    fclose(fp);
    (* results) = res;
    return (0);
}

int checkResults(void *newResults)
{
    res_t *newRes = (res_t *) newResults;

    uint biasedNumber = 16;
    uint maxScoring = 255 - biasedNumber - MATCH_SCORE;
    uint sizeOfGroup = 4;

    uint idCandidate = 0, idGroup, numResults = newRes->numResults;
    uint newCandidateScore, idIntraGroup;
    uint numGroups = DIV_CEIL(numResults, sizeOfGroup);

    uint totalFalsePositives = 0, totalGroupsFalsePositives = 0;

	for(idGroup = 0; idGroup < numGroups; ++idGroup){
		uint correctness = TRUE;

		for(idIntraGroup = 0; idIntraGroup < sizeOfGroup; ++idIntraGroup){
			idCandidate = idGroup * sizeOfGroup + idIntraGroup;
			newCandidateScore = newRes->h_results[idCandidate].score;
			if(newCandidateScore >= maxScoring) correctness = FALSE;
		}

		if(correctness == FALSE){
			totalGroupsFalsePositives++;
			totalFalsePositives += sizeOfGroup;
		}

		for(idIntraGroup = 0; idIntraGroup < sizeOfGroup; ++idIntraGroup){
			idCandidate = idGroup * sizeOfGroup + idIntraGroup;
			newRes->h_verified[idCandidate] = correctness;
		}
	}

	printf("Number of candidates benchmarked: \t\t [ %d ]\n", numResults);
	printf("Total 'False positives' REPORTED: \t\t %d \t\t(%3.2f \%)\n", totalFalsePositives, (float)totalFalsePositives/numResults * 100);
	printf("\t => Groups REPORTED: \t\t\t %d \t\t(%3.2f \%)\n", totalGroupsFalsePositives, (float)totalGroupsFalsePositives/numResults * 100);

    return(0);
}

int compareResults(void *oldResults, void *newResults)
{
    res_t *oldRes = (res_t *) oldResults;
    res_t *newRes = (res_t *) newResults;

    uint idCandidate, numResults;
    uint biasedNumber = 16;
    uint maxScoring = 255 - biasedNumber - MATCH_SCORE;
    uint oldCandidateScore, newCandidateScore;
	
	uint totalMaxScoring = 0;
	uint totalUnderMaxScoring = 0;
	uint diffScores = 0;

	numResults = MIN(oldRes->numResults, newRes->numResults);
    for(idCandidate = 0; idCandidate < numResults; idCandidate++){

		oldCandidateScore = oldRes->h_results[idCandidate].score;
		newCandidateScore = newRes->h_results[idCandidate].score;
		if (oldCandidateScore != newCandidateScore){
			oldRes->h_verified[idCandidate] = FALSE;
			diffScores++;

			if(newCandidateScore >= maxScoring) totalMaxScoring++;
			if(newCandidateScore < maxScoring) totalUnderMaxScoring++;
		}
    }

	printf("\t => Total with maxScoring: \t\t %d \t\t(%3.2f \%)\n",  totalMaxScoring, (float)totalMaxScoring/numResults * 100);
	printf("\t => Total under maxScoring: \t\t %d \t\t(%3.2f \%)\n", totalUnderMaxScoring, (float)totalUnderMaxScoring/numResults * 100);

	printf("TOTAL DIFFERENT RESULTS: \t\t\t %d \t\t(%3.2f \%)\n", diffScores, (float)diffScores/numResults * 100);
    return(0);
}

int intersectResults(void *oldResults, void *newResults)
{
    res_t *oldRes = (res_t *) oldResults;
    res_t *newRes = (res_t *) newResults;

    uint idCandidate, numResults;
    uint biasedNumber = 16;
    uint maxScoring = 255 - biasedNumber - MATCH_SCORE;

    uint oldCandidateChecksum, newCandidateChecksum;

	uint overheadAlignments   = 0;
	uint detectedAlignments   = 0;
	uint undetectedAlignments = 0;

	numResults = MIN(oldRes->numResults, newRes->numResults);
    for(idCandidate = 0; idCandidate < numResults; idCandidate++){
		oldCandidateChecksum = oldRes->h_verified[idCandidate];
		newCandidateChecksum = newRes->h_verified[idCandidate];
		if ((oldCandidateChecksum ==  FALSE) && (newCandidateChecksum == FALSE))  detectedAlignments++;
		if ((oldCandidateChecksum ==  FALSE) && (newCandidateChecksum == TRUE))  undetectedAlignments++;
		if ((oldCandidateChecksum ==  TRUE)  && (newCandidateChecksum == FALSE))  overheadAlignments++;
    }

	printf("Intersection results:\n");
	printf("\t => Total detected alignments: \t\t %d \t\t(%3.2f \%)\n", detectedAlignments, (float)detectedAlignments/numResults * 100);
	printf("\t => Total undetected alignments: \t %d \t\t(%3.2f \%)\n", undetectedAlignments, (float)undetectedAlignments/numResults * 100);
	printf("\t => Total overhead alignments: \t\t %d \t\t(%3.2f \%)\n",  overheadAlignments, (float)overheadAlignments/numResults * 100);

	return(0);
}

char *processError(int e){ 
    printf("ERROR: %d\n", e);
    switch(e) {
        case 0:  return "No error"; break; 
        case 1:  return "Error opening reference file"; break; 
        case 2:  return "Error opening queries file"; break;
		case 5:  return "Error reading candidates"; break; 
		case 6:  return "Error reading original results"; break; 
        case 30: return "Cannot open reference file"; break;
        case 31: return "Cannot allocate reference"; break;
        case 33: return "Cannot allocate queries"; break;
        case 34: return "Cannot allocate candidates"; break;
        case 32: return "Reference file isn't multifasta format"; break;
        case 37: return "Cannot open reference file on write mode"; break;
        case 42: return "Cannot open queries file"; break;
        case 43: return "Cannot allocate queries"; break;
        case 45: return "Cannot allocate results"; break;
        case 47: return "Cannot open results file for save intervals"; break;
        case 48: return "Cannot open results file for load intervals"; break;
        case 99: return "Not implemented"; break;
        default: return "Unknown error";
    }   
}

int freeResults(void *results)
{   
    res_t *res = (res_t *) results;  

    if(res->h_results != NULL){
        free(res->h_results);
        res->h_results = NULL;
    }

    if(res->h_verified != NULL){
            free(res->h_verified);
            res->h_verified = NULL;
    }
    
    return(0);
}

int main(int argc, char *argv[])
{
    void *oldResults, *newResults;
    int32_t error;
    unsigned char *oldFile = argv[1];
    unsigned char *newFile = argv[2];

    if(argc != 3){
    	printf("[INFO] This program check the correctness of the SW results \n");
        printf("Usage: ./checksum oldFile newFileToCheck\n");
        return (0);
    }

    printf("[INFO] Loading results: %s ...\n", oldFile);
    error = loadResults(oldFile, &oldResults);
    CATCH_ERROR(error);

    printf("[INFO] Loading results: %s ...\n", newFile);
    error = loadResults(newFile, &newResults);
    CATCH_ERROR(error);

    printf("[INFO] Checking results ...\n");
    checkResults(newResults);

    printf("[INFO] Comparing results ...\n");
    compareResults(oldResults, newResults);

    printf("[INFO] Intersecting results ...\n");
    intersectResults(oldResults, newResults);

    error = freeResults(oldResults);
    CATCH_ERROR(error);
    error = freeResults(newResults);
    CATCH_ERROR(error);

    return (0);
}
