/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_errors
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface to manage errors. 
 */

#ifndef PSA_ERRORS_H_
#define PSA_ERRORS_H_

#include "psa_commons.h"

#define	PSA_ERROR(error)		(psaError(error, __FILE__, __LINE__ ))

typedef enum
{
    SUCCESS,
    E_OPENING_FILE,
    E_READING_FILE,
    E_WRITING_FILE,
	E_INSUFFICIENT_MEM_GPU,
	E_ALLOCATE_MEM,
	E_INCOMPATIBLE_GPU,
	E_NO_SUPPORTED_GPUS,
	E_REFERENCE_CODING,
	E_PROGRAM_PARAMETERS,
	E_NO_MULTIFASTA_FILE,
	E_PARSING_REGPROF
} psaError_t;

void psaError(psaError_t error, const char *file, int32_t line);
char *psaGetErrorString(psaError_t error);


#endif /* PSA_ERRORS_H_ */
