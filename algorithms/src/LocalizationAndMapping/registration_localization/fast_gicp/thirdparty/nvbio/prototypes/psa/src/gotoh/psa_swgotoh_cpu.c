/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_swgotoh_cpu
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Functions for the SW-Gotoh CPU implementation using:
 *				(A) LOCAL / GLOBAL alignments
 *				(B) 8 bits per base and 32bits per cell
 *				(C) validation purposes
 */

#include "../../include/psa_pairwise.h"

#define WORD_SIZE 			32

#define MATCH_SCORE			 2
#define MISMATCH_SCORE 		-5
#define OPEN_INDEL_SCORE	-2
#define EXTEND_INDEL_SCORE	-1
#define LOCAL


#define MAX3(a,b,c)		((a >= b) ? ((a >= c) ? a : c) : ((b >= c) ? b : c))

#define BUILD_ALGORITHM	"SMITH & WATERMAN GOTOH"
#define BUILD_VERSION 	"CPU VERSION WHOLE MATRIX"
#define BUILD_TAG		"sw-gotoh.32b.cpu"

char* buildTag()
{
	return (BUILD_TAG);
}

psaError_t printBuildInfo()
{
	printf("%s \n", BUILD_ALGORITHM);
	printf("%s \n", BUILD_VERSION);
	printf("CONFIG - MATCH: %d, MISMATCH: %d, OPEN_INDEL: %d, EXTEND_INDEL: %d \n",
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

psaError_t localProcessSWG(alignments_t *alignments, uint32_t idCandidate, char *candidate, uint32_t candidateSize,
						char *query, uint32_t querySize, int32_t *H, int32_t *E, int32_t *F)
{
	int32_t sizeColumn = querySize + 1, numColumns = candidateSize + 1;
	int32_t idColumn, i, y, j;
	int32_t H_cellLeft, H_cellTop, H_cellDiag, H_cellScore;
	int32_t E_cellTop, E_cellScore;
	int32_t F_cellLeft, F_cellScore;
	int32_t W_cellScore;
	int32_t maxColumn = 0;
	char base;

	#if defined(LOCAL)
		int32_t H_maxScore = INT32_MIN;
	#else
		int32_t H_maxScore = OPEN_INDEL_SCORE + (EXTEND_INDEL_SCORE * querySize); // GLOBAL
	#endif

	#if defined(DEBUG)
		int32_t idRow;
	#endif

	// Horizontal initialization 
	for(i = 0; i < numColumns; i++){
		#if defined(LOCAL) //LOCAL
			H[i] = 0;
			F[i] = 0;
		#else //GLOBAL
			H[i] = OPEN_INDEL_SCORE + (EXTEND_INDEL_SCORE * i);
			F[i] = INT32_MIN;
		#endif
	}
	// Vertical initialization
	for(i = 0; i < sizeColumn; i++){
		#if defined(LOCAL) //LOCAL
			H[i * numColumns] = 0;
			E[i * numColumns] = 0;
		#else //GLOBAL
			E[i * numColumns] = INT32_MIN;
			H[i * numColumns] = OPEN_INDEL_SCORE + (EXTEND_INDEL_SCORE * i);
		#endif
	}
	H[0] = 0;

	// Compute Score SW-GOTOH
	for(idColumn = 1; idColumn < numColumns; idColumn++){
		base = candidate[idColumn - 1];
		#if defined(DEBUG)
			int32_t H_maxColumn = 0;
		#endif
		for(y = 1; y < sizeColumn; y++){
			W_cellScore = (base == query[y - 1]) ? MATCH_SCORE : MISMATCH_SCORE;

			H_cellLeft 	= H[y * numColumns + (idColumn - 1)];
			H_cellTop 	= H[(y - 1) * numColumns + idColumn];
			H_cellDiag 	= H[(y - 1) * numColumns + (idColumn - 1)];

			E_cellTop	= E[(y - 1) * numColumns + idColumn];
			F_cellLeft  = F[y * numColumns + (idColumn - 1)];

			E_cellScore = MAX(E_cellTop + EXTEND_INDEL_SCORE,
							  H_cellTop + OPEN_INDEL_SCORE);
			F_cellScore = MAX(F_cellLeft + EXTEND_INDEL_SCORE,
							  H_cellLeft + OPEN_INDEL_SCORE);
			H_cellScore = MAX3(E_cellScore, F_cellScore, H_cellDiag + W_cellScore);
			H_cellScore = MAX(H_cellScore, 0);

			E[y * numColumns + idColumn] = E_cellScore;
			F[y * numColumns + idColumn] = F_cellScore;
			H[y * numColumns + idColumn] = H_cellScore;

			#if defined(DEBUG)
				H_maxColumn = MAX(H_cellScore, H_maxColumn);
			#endif

			if(H_cellScore >= H_maxScore){
				H_maxScore = H_cellScore;
				maxColumn  = idColumn - 1;
			}
		}
	}

	#if defined(DEBUG)
		if(idCandidate == 0){
			printf("\n E (D): \n");
			for(idColumn = 0; idColumn < sizeColumn; idColumn++){
				if(idColumn != 0) printf("\t %c:", query[idColumn - 1]);
				for(idRow = 0; idRow < numColumns; idRow++){
					printf("%4d,", E[(idColumn * numColumns) + idRow]);
				}
				printf("\n");
			}
			printf("\n F (I): \n");
			for(idColumn = 0; idColumn < sizeColumn; idColumn++){
				if(idColumn != 0) printf("\t %c:", query[idColumn - 1]);
				for(idRow = 0; idRow < numColumns; idRow++){
					printf("%4d,", F[(idColumn * numColumns) + idRow]);
				}
				printf("\n");
			}
			printf("\n H (M): \n");
			for(idColumn = 0; idColumn < sizeColumn; idColumn++){
				if(idColumn != 0) printf("\t %c:", query[idColumn - 1]);
				for(idRow = 0; idRow < numColumns; idRow++){
					printf("%4d,", H[(idColumn * numColumns) + idRow]);
				}
				printf("\n");
			}
		}
	#endif

	//alignments->h_results[idCandidate].column = maxColumn; 
	alignments->h_results[idCandidate].column = 0; 
	alignments->h_results[idCandidate].score  = H_maxScore;

	return(SUCCESS);
}

psaError_t processPairwiseStream(sequences_t *candidates, sequences_t *queries, alignments_t *alignments)
{
	uint32_t idCandidate, words;
	uint32_t scoreBP, scoreSW;
	char *S1, *S2;
	uint32_t candidateSize, querySize;
	int32_t *H, *E, *F;

	words = DIV_CEIL(candidates->h_size[0], UINT32_LENGTH);
	H = (int32_t *) malloc((candidates->h_size[0] + 1) * (queries->h_size[0] + 1) * sizeof(int32_t));
	if (H == NULL) return (E_ALLOCATE_MEM);
	E = (int32_t *) malloc((candidates->h_size[0] + 1) * (queries->h_size[0] + 1) * sizeof(int32_t));
	if (E == NULL) return (E_ALLOCATE_MEM);
	F = (int32_t *) malloc((candidates->h_size[0] + 1) * (queries->h_size[0] + 1) * sizeof(int32_t));
	if (F == NULL) return (E_ALLOCATE_MEM);

	for (idCandidate = 0; idCandidate < candidates->num; ++idCandidate)
	{
		S1 = &candidates->h_ASCII[candidates->h_ASCIIposition[idCandidate]];
		S2 = &queries->h_ASCII[queries->h_ASCIIposition[alignments->h_info[idCandidate]]];
		candidateSize = candidates->h_size[idCandidate];
		querySize = queries->h_size[alignments->h_info[idCandidate]];

		localProcessSWG(alignments, idCandidate, S1, candidateSize, S2, querySize, H, E, F);
	}

	free(H);
	free(F);
	free(E);
	
	return (SUCCESS);
}







