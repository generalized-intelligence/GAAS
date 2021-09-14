/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_sequences
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface to manage / transform string sequence formats.
 */

#ifndef PSA_SEQUENCES_H_
#define PSA_SEQUENCES_H_

#include <string.h>
#include "psa_commons.h"
#include "psa_errors.h"

#define PEQ_ENTRY_LENGTH	128
#define PEQ_SUB_ENTRIES		(PEQ_ENTRY_LENGTH / UINT32_LENGTH)
#define RAW_16B_LENGTH		4

#define RAW_4B_LENGTH 		2


typedef enum
{
  SEQ_VOID      	= UINT32_LOW_ONE << 0,
  SEQ_PEQ_4x32b 	= UINT32_LOW_ONE << 1,
  SEQ_RAW   		= UINT32_LOW_ONE << 2,
  SEQ_ASCII 	  	= UINT32_LOW_ONE << 3,
  SEQ_HLF_RAW  		= UINT32_LOW_ONE << 4,
  SEQ_REF_POS		= UINT32_LOW_ONE << 5
} seq_format_t;

typedef struct {
	uint32_t 	bitmap[NUM_BASES][PEQ_SUB_ENTRIES];
} PEQEntry_t;

typedef uint64_t 	RAWEntry_t;
typedef uint32_t 	RAWHlfEntry_t;

typedef char 		ASCIIEntry_t;

typedef struct {
    /* sequences info */
    uint32_t 		num;
    seq_format_t	formats;
    /* Reference Positions */
    uint32_t 		*h_refPosition;
    uint32_t 		*d_refPosition;
	/* PEQ structures */
	uint32_t 		numPEQEntries;
	PEQEntry_t		*h_PEQ;
	PEQEntry_t		*d_PEQ;
	uint32_t		*h_PEQposition;
	uint32_t		*d_PEQposition;
	/* RAW structures */
	uint32_t 		numRAWEntries;
	RAWEntry_t		*h_RAW;
	RAWEntry_t		*d_RAW;
	uint32_t		*h_RAWposition;
	uint32_t		*d_RAWposition;
	/* Half RAW structures (smaller entries)*/
	uint32_t 		numRAWHlfEntries;
	RAWHlfEntry_t	*h_HlfRAW;
	RAWHlfEntry_t	*d_HlfRAW;
	uint32_t		*h_HlfRAWposition;
	uint32_t		*d_HlfRAWposition;
	/* ASCII structures (debug) */
	uint32_t 		numASCIIEntries;
	ASCIIEntry_t	*h_ASCII;
	ASCIIEntry_t	*d_ASCII;
	uint32_t		*h_ASCIIposition;
	uint32_t		*d_ASCIIposition;
	/* Common data to all formats */
	uint32_t 		*h_size;
    uint32_t 		*d_size;
} sequences_t;

psaError_t 		initSequences(sequences_t **sequences, uint32_t numSequences);
psaError_t 		allocateSequences(sequences_t *sequences, uint32_t numEntries, seq_format_t SEQ_FORMAT);

uint32_t 		ASCIIto4bits(char base);
uint32_t 		ASCIItoIndex(char base);
psaError_t 		ASCIItoPEQ_32bits(char *h_ASCII, uint32_t idSubPEQ, PEQEntry_t *PEQEntry, int numBases);
psaError_t 		ASCIItoPEQ_31bits(char *h_ASCII, uint32_t idSubPEQ, PEQEntry_t *PEQEntry, int numBases);
uint64_t 		ASCIItoRAW_64bits(unsigned char base);
uint32_t 		ASCIItoRAW_32bits(unsigned char base);
char			ASCIItoUpperCase(char base);

psaError_t 		sequences_positionASCIItoPEQ_128bits(uint32_t *h_PEQposition, uint32_t *h_size, uint32_t num);
psaError_t 		sequences_recoverASCII(uint32_t position, uint32_t *h_size, uint32_t num);

uint32_t 		sequences_totalEntriesRAW(uint32_t *h_size, uint32_t num, const uint32_t baseSize);
uint32_t 		sequences_totalEntriesHlfRAW(uint32_t *h_size, uint32_t num, const uint32_t baseSize);
uint32_t 		sequences_totalEntriesPEQ_128bits(uint32_t *h_size, uint32_t num);
uint32_t 		sequences_totalEntriesPEQ_4x31bits(uint32_t *h_size, uint32_t num);

psaError_t 		sequenceASCIItoPEQ_4x32bits(char *h_ASCII, uint32_t sizeSequence, PEQEntry_t *h_PEQ);
psaError_t 		sequenceASCIItoPEQ_4x31bits(char *h_ASCII, uint32_t sizeSequence, PEQEntry_t *h_PEQ);
psaError_t 		sequenceASCIItoRAW_4x64bits(const char *h_ASCII, int32_t size, RAWEntry_t *h_RAW);
psaError_t  	sequenceASCIItoRAW_2x32bits(const char *h_ASCII, int32_t size, RAWHlfEntry_t *h_HlfRAW);


psaError_t 		loadSequenceRAW_64bits(const char *h_ASCII, int32_t size, char *h_RAW, int32_t numRAWEntries);
psaError_t 		loadSequenceMFASTA(FILE *fp, uint32_t **h_size, char **h_ASCII, uint32_t **h_ASCIIposition, seq_format_t *formats);

psaError_t 		saveSequences(FILE *fp, sequences_t *sequences);
psaError_t 		loadSequences(FILE *fp, sequences_t *sequences);
psaError_t 		freeSequences(sequences_t **sequences);

#endif /* PSA_SEQUENCES_H_ */













