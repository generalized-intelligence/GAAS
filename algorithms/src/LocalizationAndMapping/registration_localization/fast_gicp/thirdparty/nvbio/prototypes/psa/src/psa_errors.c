/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_errors
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface to manage errors. 
 */

#include "../include/psa_errors.h"

char *psaGetErrorString(psaError_t error)
{
    switch(error) {
        case E_OPENING_FILE:          return "PSA - Error: opening file"; break;
        case E_READING_FILE:          return "PSA - Error: reading file"; break;
        case E_WRITING_FILE:          return "PSA - Error: reading file"; break;
        case E_INSUFFICIENT_MEM_GPU:	return "PSA - Error: there aren't enough GPU memory space"; break;
        case E_ALLOCATE_MEM:          return "PSA - Error: allocating data"; break;
        case E_INCOMPATIBLE_GPU:      return "PSA - Error: incompatible GPU (old CC version)"; break;
        case E_REFERENCE_CODING:      return "PSA - Error: reference coding not supported"; break;
        case E_NO_MULTIFASTA_FILE:    return "PSA - Error: reference file isn't multifasta format"; break;
        case E_PARSING_REGPROF:       return "PSA - Error: parsing a candidate from GEM regions profile"; break;
        default:                      return "PSA - Unknown error";
    }
}

void psaError(psaError_t error, const char *file, int32_t line) 
{
   	if (error != 0) {
      		fprintf(stderr, "%s in %s at line %d\n", psaGetErrorString(error),  file, line);
       		exit(EXIT_FAILURE);
   	}
}
