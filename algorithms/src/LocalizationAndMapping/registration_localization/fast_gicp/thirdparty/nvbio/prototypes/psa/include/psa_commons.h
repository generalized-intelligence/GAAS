/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_commons
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: Common definitions and functions.
 */

#ifndef PSA_COMMONS_H_
#define PSA_COMMONS_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/********************************
Common constants for Device & Host
*********************************/

#define UINT32_ZEROS		0x00000000u
#define UINT32_ONES  		0xFFFFFFFFu
#define UINT32_LOW_ONE  	0x00000001u
#define UINT32_HIGH_ONE		0x80000000u
#define	UINT32_SIZE			4
#define	UINT32_LENGTH		32
#define	UINT64_SIZE			8
#define	UINT64_LENGTH		64

#define	BITPAL_INLINE		inline

/* Functions inline */
#define DIV_CEIL(NUMERATOR,DENOMINATOR) (((NUMERATOR)+((DENOMINATOR)-1))/(DENOMINATOR))
#define ROUND(NUM) ((int)((NUM) < 0 ? ((NUM) - 0.5) : ((NUM) + 0.5)))
#define MIN(NUM_A,NUM_B) (((NUM_A) < (NUM_B)) ? (NUM_A) : (NUM_B))
#define MAX(NUM_A,NUM_B) (((NUM_A) >= (NUM_B)) ? (NUM_A) : (NUM_B))

/* Conversion utils */
#define CONVERT_B_TO_KB(NUM) ((NUM)/(1024))
#define CONVERT_B_TO_MB(NUM) ((NUM)/(1024*1024))
#define CONVERT_B_TO_GB(NUM) ((NUM)/(1024*1024*1024))
#define CONVERT_MB_TO_B(NUM) ((NUM)*1024*1024)

/* System */
#define FILE_SIZE_LINES		20000

/* Encoded DNA Nucleotides */
#define NUM_BASES			5
#define ENC_DNA_CHAR_A		0LL
#define ENC_DNA_CHAR_C		1LL
#define ENC_DNA_CHAR_G		2LL
#define ENC_DNA_CHAR_T		3LL

#define ENC_DNA_CHAR_N		4LL
#define ENC_DNA_CHAR_SEP	5LL
#define ENC_DNA_CHAR_JUMP	6LL

#endif /* PSA_COMMONS_H_ */

