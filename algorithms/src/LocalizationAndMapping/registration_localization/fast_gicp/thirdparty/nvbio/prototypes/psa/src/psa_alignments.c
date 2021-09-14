/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_alignments
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API functions to manage the alignments (sink and score)
 */

#include "../include/psa_alignments.h"

psaError_t initAlignments(alignments_t **alignments, uint32_t numAlignments)
{
    (* alignments) = (alignments_t *) malloc(sizeof(alignments_t));
    	if((* alignments) == NULL) return (E_ALLOCATE_MEM);

 	/* Alignment data */
	(* alignments)->num = numAlignments;
	(* alignments)->numReordered = 0;
 	/* Alignment structures */
	(* alignments)->h_info = 0;
	(* alignments)->d_info = 0;
	(* alignments)->h_results = NULL;
	(* alignments)->d_results = NULL;
 	/* Binning by size structures */
	(* alignments)->h_reorderResults = NULL;
	(* alignments)->d_reorderResults = NULL;

    return (SUCCESS);
}

psaError_t allocateAlignments(alignments_t *alignments)
{
    if (alignments == NULL) return (E_ALLOCATE_MEM);

    alignments->h_info = (alignmentInfo_t *) malloc(alignments->num * sizeof(alignmentInfo_t));
	if (alignments->h_info == NULL) return (E_ALLOCATE_MEM);
	
    alignments->h_results = (alignmentEntry_t *) malloc(alignments->num * sizeof(alignmentEntry_t));
	if (alignments->h_results == NULL) return (E_ALLOCATE_MEM);

    return (SUCCESS);
}

psaError_t loadAlignments(FILE *fp, alignments_t *alignments)
{
	uint32_t result;

	result = fread(&alignments->num, sizeof(uint32_t), 1, fp);
    if (result != 1) return(E_WRITING_FILE);

    alignments->h_info = (alignmentInfo_t *) malloc(alignments->num * sizeof(alignmentInfo_t));
	if (alignments->h_info == NULL) return (E_ALLOCATE_MEM);
	result = fread(alignments->h_info, sizeof(alignmentInfo_t), alignments->num, fp);
	if (result != alignments->num) return(E_WRITING_FILE);
	
    alignments->h_results = (alignmentEntry_t *) malloc(alignments->num * sizeof(alignmentEntry_t));
	if (alignments->h_results == NULL) return (E_ALLOCATE_MEM);
	result = fread(alignments->h_results, sizeof(alignmentEntry_t), alignments->num, fp);
	if (result != alignments->num) return(E_WRITING_FILE);

	return (SUCCESS);
}

psaError_t saveAlignments(FILE *fp, alignments_t *alignments)
{
	uint32_t result;

	result = fwrite(&alignments->num, sizeof(uint32_t), 1, fp);
    if (result != 1) return(E_WRITING_FILE);
	result = fwrite(alignments->h_info, sizeof(alignmentInfo_t), alignments->num, fp);
	if (result != alignments->num) return(E_WRITING_FILE);
	result = fwrite(alignments->h_results, sizeof(alignmentEntry_t), alignments->num, fp);
	if (result != alignments->num) return(E_WRITING_FILE);

	return (SUCCESS);
}

psaError_t saveASCIIAlignments(FILE *fp, alignments_t *alignments)
{
	char textLine[256];
	uint32_t error, idAlignment;

	sprintf(textLine, "%u\n", alignments->num);
	fputs(textLine, fp);
	for(idAlignment = 0; idAlignment < alignments->num; idAlignment++){
		sprintf(textLine, "%u %d %u\n", idAlignment, alignments->h_results[idAlignment].score, alignments->h_results[idAlignment].column);
		fputs(textLine, fp);
	}

	return(SUCCESS);
}

psaError_t freeAlignments(alignments_t **alignments)
{   
	if((* alignments) != NULL){

		if((* alignments)->h_info != NULL){
	        free((* alignments)->h_info);
	        (* alignments)->h_info = NULL;
	    }

	    if((* alignments)->h_results != NULL){
	        free((* alignments)->h_results);
	        (* alignments)->h_results = NULL;
	    }

	    if((* alignments)->h_reorderResults != NULL){
	        free((* alignments)->h_reorderResults);
	        (* alignments)->h_reorderResults = NULL;
	    }
	    
    	free((* alignments));
    	(* alignments) = NULL;
    }

	return(SUCCESS);
}

