/*
 * PROJECT: Pairwise sequence alignments on GPU
 * FILE: psa_sequences
 * AUTHOR(S): Alejandro Chacon <alejandro.chacon@uab.es>
 * DESCRIPTION: API Interface to manage / transform string sequence formats.
 */

#include "../include/psa_sequences.h"

psaError_t initSequences(sequences_t **sequences, uint32_t numSequences)
{
    (* sequences) = (sequences_t *) malloc(sizeof(sequences_t));
    if((* sequences) == NULL) return (E_ALLOCATE_MEM);

	/* sequences info */
	(* sequences)->num = numSequences;
	(* sequences)->formats = SEQ_VOID;
	/* Ref absolute position */
	(* sequences)->h_refPosition = NULL;
	(* sequences)->d_refPosition = NULL;
	/* PEQ structures */
	(* sequences)->numPEQEntries = 0;
	(* sequences)->h_PEQ = NULL;
	(* sequences)->d_PEQ = NULL;
	(* sequences)->h_PEQposition = NULL;
	(* sequences)->d_PEQposition = NULL;
	/* RAW structures */
	(* sequences)->numRAWEntries = 0;
	(* sequences)->h_RAW = NULL;
	(* sequences)->d_RAW = NULL;
	(* sequences)->h_RAWposition = NULL;
	(* sequences)->d_RAWposition = NULL;
	/* ASCII structures (debug) */
	(* sequences)->numASCIIEntries = 0;
	(* sequences)->h_ASCII = NULL;
	(* sequences)->d_ASCII = NULL;
	(* sequences)->h_ASCIIposition = NULL;
	(* sequences)->d_ASCIIposition = NULL;
	/* Common data to all formats */
	(* sequences)->h_size = NULL;
	(* sequences)->d_size = NULL;

    return (SUCCESS);
}

psaError_t allocateSequences(sequences_t *sequences, uint32_t numEntries, seq_format_t SEQ_FORMAT)
{
    if (sequences == NULL)
     return (E_ALLOCATE_MEM);

    if(SEQ_REF_POS & SEQ_FORMAT){
    	sequences->h_refPosition = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
    	if (sequences->h_refPosition == NULL) PSA_ERROR(E_ALLOCATE_MEM);
    }

	if(SEQ_PEQ_4x32b & SEQ_FORMAT){
		/* PEQ structures */
		sequences->numPEQEntries = numEntries;

		if(sequences->h_PEQ != NULL) free(sequences->h_PEQ);
		sequences->h_PEQ = (PEQEntry_t *) malloc(sequences->numPEQEntries * sizeof(PEQEntry_t));
		if (sequences->h_PEQ == NULL) return (E_ALLOCATE_MEM);

		if(sequences->h_PEQposition != NULL) free(sequences->h_PEQposition);
    	sequences->h_PEQposition = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
		if (sequences->h_PEQposition == NULL) return (E_ALLOCATE_MEM);
	}

	if(SEQ_RAW & SEQ_FORMAT){
		/* RAW structures */
		sequences->numRAWEntries = numEntries;

		if(sequences->h_RAW != NULL) free(sequences->h_RAW);
    	sequences->h_RAW = (RAWEntry_t *) malloc(sequences->numRAWEntries * sizeof(RAWEntry_t));
		if (sequences->h_RAW == NULL) return (E_ALLOCATE_MEM);

		if(sequences->h_RAWposition != NULL) free(sequences->h_RAWposition);
    	sequences->h_RAWposition = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
		if (sequences->h_RAWposition == NULL) return (E_ALLOCATE_MEM);
	}

	if(SEQ_ASCII & SEQ_FORMAT){
		/* ASCII structures (debug) */
		sequences->numASCIIEntries = numEntries;

		if(sequences->h_ASCII != NULL) free(sequences->h_ASCII);
    	sequences->h_ASCII = (ASCIIEntry_t *) malloc(sequences->numASCIIEntries * sizeof(ASCIIEntry_t));
		if (sequences->h_ASCII == NULL) return (E_ALLOCATE_MEM);
    	
    	if(sequences->h_ASCIIposition != NULL) free(sequences->h_ASCIIposition);
    	sequences->h_ASCIIposition = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
		if (sequences->h_ASCIIposition == NULL) return (E_ALLOCATE_MEM);
	}

	if(sequences->formats == SEQ_VOID){
		/* Common data to all formats */
		if(sequences->h_size != NULL) free(sequences->h_size);
		sequences->h_size = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
		if (sequences->h_size == NULL) return (E_ALLOCATE_MEM);
	}

	/* sequences info */
	sequences->formats |= SEQ_FORMAT;

    return (SUCCESS);
}

uint32_t ASCIIto4bits(char base)
{
	switch(base)
	{
    	case 'A':
    	case 'a':
    	    return(1);
    	case 'C':
    	case 'c':
    	    return(2);
    	case 'G':
    	case 'g':
    	    return(4);
    	case 'T':
    	case 't':
    	    return(8);
    	default :
    	    return(16);
	}
}

uint32_t ASCIItoIndex(char base)
{
	switch(base)
	{
    	case 'A':
    	case 'a':
    	    return(0);
    	case 'C':
    	case 'c':
    	    return(1);
    	case 'G':
    	case 'g':
    	    return(2);
    	case 'T':
    	case 't':
    	    return(3);
    	default :
    	    return(4);
	}
}

char ASCIItoUpperCase(char base)
{
	switch(base)
	{
    	case 'A':
    	case 'a':
    	    return('A');
    	case 'C':
    	case 'c':
    	    return('C');
    	case 'G':
    	case 'g':
    	    return('G');
    	case 'T':
    	case 't':
    	    return('T');
    	default :
    	    return('A');
	}
}

psaError_t ASCIItoPEQ_32bits(char *h_ASCII, uint32_t idSubPEQ, PEQEntry_t *PEQEntry, int numBases)
{
	int i; 
	uint32_t indexBase;
	uint32_t bitmapA 	= 0;
	uint32_t bitmapC 	= 0;
	uint32_t bitmapG 	= 0;
	uint32_t bitmapT 	= 0;
	uint32_t bitmapN 	= 0;

	for(i = 0; i < numBases; i++){
		indexBase = ASCIIto4bits(h_ASCII[i]);
		bitmapA |= ((indexBase & 1 )      ) << i;
		bitmapC |= ((indexBase & 2 )  >> 1) << i;
		bitmapG |= ((indexBase & 4 )  >> 2) << i;
		bitmapT |= ((indexBase & 8 )  >> 3) << i;
		bitmapN |= ((indexBase & 16)  >> 4) << i;
	}

	PEQEntry->bitmap[0][idSubPEQ] = bitmapA;
	PEQEntry->bitmap[1][idSubPEQ] = bitmapC;
	PEQEntry->bitmap[2][idSubPEQ] = bitmapG;
	PEQEntry->bitmap[3][idSubPEQ] = bitmapT;
	PEQEntry->bitmap[4][idSubPEQ] = bitmapN;

	return (SUCCESS);
}

psaError_t ASCIItoPEQ_31bits(char *h_ASCII, uint32_t idSubPEQ, PEQEntry_t *PEQEntry, int numBases)
{
	int i; 
	uint32_t indexBase;
	uint32_t bitmap[5] 	= {0,0,0,0,0};
	uint32_t bitmask = 1;

	for(i = 0; i < numBases; i++){
		indexBase = ASCIItoIndex(h_ASCII[i]);
		bitmask <<= 1;
		bitmap[indexBase] |= bitmask;
	}

	PEQEntry->bitmap[0][idSubPEQ] = bitmap[0] >> 1;
	PEQEntry->bitmap[1][idSubPEQ] = bitmap[1] >> 1;
	PEQEntry->bitmap[2][idSubPEQ] = bitmap[2] >> 1;
	PEQEntry->bitmap[3][idSubPEQ] = bitmap[3] >> 1;
	PEQEntry->bitmap[4][idSubPEQ] = bitmap[4] >> 1;

	return (SUCCESS);
}

psaError_t sequences_positionASCIItoPEQ_128bits(uint32_t *h_PEQposition, uint32_t *h_size, uint32_t num)
{
	uint32_t idSequence, sizeSequence, totalEntriesPEQ = 0;

	const uint32_t BASES_PER_PEQ_ENTRY = PEQ_SUB_ENTRIES * UINT32_LENGTH;

	for(idSequence = 0; idSequence < num; ++idSequence){
		h_PEQposition[idSequence] = totalEntriesPEQ;
		sizeSequence = h_size[idSequence];
		totalEntriesPEQ += DIV_CEIL(sizeSequence, BASES_PER_PEQ_ENTRY);
	}

	return (SUCCESS);
}

uint32_t sequences_totalEntriesPEQ_4x31bits(uint32_t *h_size, uint32_t num)
{
	uint32_t sizeSequence, idSequence, totalEntriesPEQ = 0;

	const uint32_t BASES_PER_PEQ_ENTRY = PEQ_SUB_ENTRIES * (UINT32_LENGTH - 1);

	for(idSequence = 0; idSequence < num; ++idSequence){
		sizeSequence = h_size[idSequence];
		totalEntriesPEQ += DIV_CEIL(sizeSequence, BASES_PER_PEQ_ENTRY);
	}

	return (totalEntriesPEQ);
}

uint32_t sequences_totalEntriesRAW(uint32_t *h_size, uint32_t num, const uint32_t baseSize)
{
	uint32_t sizeSequence, idSequence, totalEntriesRAW = 0;

	const uint32_t RAW_BASES_PER_ENTRY = UINT64_LENGTH / baseSize;

	for(idSequence = 0; idSequence < num; ++idSequence){
		sizeSequence = h_size[idSequence];
		totalEntriesRAW += DIV_CEIL(sizeSequence, RAW_BASES_PER_ENTRY);
	}

	return (totalEntriesRAW);
}

uint32_t sequences_totalEntriesHlfRAW(uint32_t *h_size, uint32_t num, const uint32_t baseSize)
{
	uint32_t sizeSequence, idSequence, totalEntriesHlfRAW = 0;

	const uint32_t RAW_BASES_PER_ENTRY = UINT32_LENGTH / baseSize;

	for(idSequence = 0; idSequence < num; ++idSequence){
		sizeSequence = h_size[idSequence];
		totalEntriesHlfRAW += DIV_CEIL(sizeSequence, RAW_BASES_PER_ENTRY);
	}

	return (totalEntriesHlfRAW);
}


psaError_t sequenceASCIItoPEQ_4x32bits(char *h_ASCII, uint32_t sizeSequence, PEQEntry_t *h_PEQ)
{
	int32_t pos, idSequence, intraSequence = 0;
	int32_t processedBases = 0;
	int32_t idSubPEQ;
	int32_t intraBasesProcessed, numBases;

	//const uint32_t PEQ_SUB_ENTRIES = 4;
	const uint32_t BASES_PER_PEQ_ENTRY = PEQ_SUB_ENTRIES * UINT32_LENGTH;

	for(pos = 0; pos < sizeSequence; pos += BASES_PER_PEQ_ENTRY){
		for(idSubPEQ = 0; idSubPEQ < PEQ_SUB_ENTRIES; ++idSubPEQ){
			intraBasesProcessed = UINT32_LENGTH * idSubPEQ;
			numBases = MIN(sizeSequence - (pos + intraBasesProcessed), UINT32_LENGTH);
			numBases = (numBases < 0) ? 0 : numBases; 
			ASCIItoPEQ_32bits(h_ASCII + processedBases + pos + intraBasesProcessed, idSubPEQ, h_PEQ + intraSequence, numBases);
		}
		intraSequence++;
	}

	return (SUCCESS);
}

psaError_t sequenceASCIItoPEQ_4x31bits(char *h_ASCII, uint32_t sizeSequence, PEQEntry_t *h_PEQ)
{
	int32_t pos, idSequence, intraSequence = 0;
	int32_t processedBases = 0;
	int32_t idSubPEQ;
	int32_t intraBasesProcessed, numBases;

	//const uint32_t PEQ_SUB_ENTRIES = 4;
	const uint32_t BASES_PER_PEQ_ENTRY = PEQ_SUB_ENTRIES * (UINT32_LENGTH - 1);

	for(pos = 0; pos < sizeSequence; pos += BASES_PER_PEQ_ENTRY){
		for(idSubPEQ = 0; idSubPEQ < PEQ_SUB_ENTRIES; ++idSubPEQ){
			intraBasesProcessed = (UINT32_LENGTH - 1) * idSubPEQ;
			numBases = MIN(sizeSequence - (pos + intraBasesProcessed), (UINT32_LENGTH - 1));
			numBases = (numBases < 0) ? 0 : numBases; 
			ASCIItoPEQ_31bits(h_ASCII + processedBases + pos + intraBasesProcessed, idSubPEQ, h_PEQ + intraSequence, numBases);
		}
		intraSequence++;
	}

	return (SUCCESS);
}


/*
psaError_t sequences_ASCIItoPEQ_4x32bits(char *h_ASCII, PEQEntry_t *h_PEQ, uint32_t *h_PEQposition, uint32_t *h_size, uint32_t num)
{
	int32_t pos, idSequence, intraSequence;
	int32_t processedBases = 0;
	int32_t startSequenceEntry, idSubPEQ;
	int32_t intraBasesProcessed, sizeSequence, numBases;

	//const uint32_t PEQ_SUB_ENTRIES = 4;
	const uint32_t BASES_PER_PEQ_ENTRY = PEQ_SUB_ENTRIES * UINT32_LENGTH;

	for(idSequence = 0; idSequence < num; ++idSequence){
		intraSequence = 0;
		startSequenceEntry = h_PEQposition[idSequence];
		sizeSequence = h_size[idSequence];
		for(pos = 0; pos < sizeSequence; pos += BASES_PER_PEQ_ENTRY){
			for(idSubPEQ = 0; idSubPEQ < PEQ_SUB_ENTRIES; ++idSubPEQ){
				intraBasesProcessed = UINT32_LENGTH * idSubPEQ;
				numBases = MIN(sizeSequence - (pos + intraBasesProcessed), UINT32_LENGTH);
				numBases = (numBases < 0) ? 0 : numBases; 
				ASCIItoPEQ_32bits(h_ASCII + processedBases + pos + intraBasesProcessed, idSubPEQ, h_PEQ + startSequenceEntry + intraSequence, numBases);
			}
			intraSequence++;
		}
		processedBases += sizeSequence;
	}

	return (SUCCESS);
}*/

uint64_t ASCIItoRAW_64bits(unsigned char base)
{
	switch(base)
	{
    	case 'A':
    	case 'a':
    	    return(ENC_DNA_CHAR_A);
    	case 'C':
    	case 'c':
    	    return(ENC_DNA_CHAR_C << (UINT64_LENGTH - RAW_16B_LENGTH));
    	case 'G':
    	case 'g':
    	    return(ENC_DNA_CHAR_G << (UINT64_LENGTH - RAW_16B_LENGTH));
    	case 'T':
    	case 't':
    	    return(ENC_DNA_CHAR_T << (UINT64_LENGTH - RAW_16B_LENGTH));
    	default :
    	    return(ENC_DNA_CHAR_N << (UINT64_LENGTH - RAW_16B_LENGTH));
	}
}

uint32_t ASCIItoRAW_32bits(unsigned char base)
{
	switch(base)
	{
    	case 'A':
    	case 'a':
    	    return(ENC_DNA_CHAR_A);
    	case 'C':
    	case 'c':
    	    return(ENC_DNA_CHAR_C << (UINT32_LENGTH - RAW_4B_LENGTH));
    	case 'G':
    	case 'g':
    	    return(ENC_DNA_CHAR_G << (UINT32_LENGTH - RAW_4B_LENGTH));
    	case 'T':
    	case 't':
    	    return(ENC_DNA_CHAR_T << (UINT32_LENGTH - RAW_4B_LENGTH));
    	default :
    	    return(ENC_DNA_CHAR_A << (UINT32_LENGTH - RAW_4B_LENGTH));
	}
}

psaError_t sequenceASCIItoRAW_4x64bits(const char *h_ASCII, int32_t size, RAWEntry_t *h_RAW)
{
	RAWEntry_t indexBase, bitmap;
	uint32_t idEntry, i, sequencePosition;
	unsigned char base;

	//4 bits per character using 64 bits
	const uint32_t RAW_BASES_PER_ENTRY = UINT64_LENGTH / RAW_16B_LENGTH;
	const uint32_t numRAWEntries = DIV_CEIL(size, RAW_BASES_PER_ENTRY);

	for(idEntry = 0; idEntry < numRAWEntries; ++idEntry){
		bitmap = 0;
		for(i = 0; i < RAW_BASES_PER_ENTRY; ++i){
			sequencePosition = (idEntry * RAW_BASES_PER_ENTRY) + i;
			if (sequencePosition < size) base = h_ASCII[sequencePosition];
				else base = 'N'; //filling reference padding
			indexBase = ASCIItoRAW_64bits(base);
			bitmap = (bitmap >> RAW_16B_LENGTH) | indexBase;
		}
		h_RAW[sequencePosition / RAW_BASES_PER_ENTRY] = bitmap;
	}
	return (SUCCESS);
}

psaError_t sequenceASCIItoRAW_2x32bits(const char *h_ASCII, int32_t size, RAWHlfEntry_t *h_HlfRAW)
{
	RAWHlfEntry_t indexBase, bitmap;
	uint32_t idEntry, i, sequencePosition;
	unsigned char base;

	//2 bits per character using 32 bits
	const uint32_t RAW_BASES_PER_ENTRY = UINT32_LENGTH / RAW_4B_LENGTH;
	const uint32_t numRAWEntries = DIV_CEIL(size, RAW_BASES_PER_ENTRY);

	for(idEntry = 0; idEntry < numRAWEntries; ++idEntry){
		bitmap = 0;
		for(i = 0; i < RAW_BASES_PER_ENTRY; ++i){
			sequencePosition = (idEntry * RAW_BASES_PER_ENTRY) + i;
			if (sequencePosition < size) base = h_ASCII[sequencePosition];
				else base = 'A'; //filling reference padding
			indexBase = ASCIItoRAW_32bits(base);
			bitmap = (bitmap >> RAW_4B_LENGTH) | indexBase;
		}
		h_HlfRAW[sequencePosition / RAW_BASES_PER_ENTRY] = bitmap;
	}
	return (SUCCESS);
}



psaError_t loadSequenceMFASTA(FILE *fp, uint32_t **h_size, char **h_ASCII, uint32_t **h_ASCIIposition, seq_format_t *formats)
{
	char lineFile[FILE_SIZE_LINES];
	uint32_t sizeFile = 0, position = 0, numGenomes = 1;
	int32_t charsRead = 0;

	fseek(fp, 0L, SEEK_END);
	sizeFile = ftell(fp);
	rewind(fp);

	(* h_ASCII) = (char *) malloc(sizeFile * sizeof(char));
	if ((* h_ASCII) == NULL) return (E_ALLOCATE_MEM);

	(* h_size) = (uint32_t *) malloc(numGenomes * sizeof(uint32_t));
	if ((* h_size) == NULL) return (E_ALLOCATE_MEM);

	(* h_ASCIIposition) = (uint32_t *) malloc(numGenomes * sizeof(uint32_t));
	if ((* h_ASCIIposition) == NULL) return (E_ALLOCATE_MEM);

	if ((fgets(lineFile, FILE_SIZE_LINES, fp) == NULL) || (lineFile[0] != '>'))
		return (E_NO_MULTIFASTA_FILE);

	while((!feof(fp)) && (fgets(lineFile, FILE_SIZE_LINES, fp) != NULL)){
		if (lineFile[0] != '>'){
			charsRead = strlen(lineFile);
			if(charsRead) charsRead--;
			memcpy(((* h_ASCII) + position), lineFile, charsRead);
			position += charsRead;
		}
	}

	(* formats) |= SEQ_ASCII;
	(* h_size)[0] = position;
	(* h_ASCIIposition)[0] = 0;

	return (SUCCESS);
}

psaError_t saveSequences(FILE *fp, sequences_t *sequences)
{
	uint32_t result;

	if(sequences->formats != SEQ_VOID){

		result = fwrite(&sequences->num, sizeof(uint32_t), 1, fp);
        if (result != 1) return(E_WRITING_FILE);
    	result = fwrite(&sequences->formats, sizeof(seq_format_t), 1, fp);
        if (result != 1) return(E_WRITING_FILE);

	    result = fwrite(sequences->h_size, sizeof(uint32_t), sequences->num, fp);
	    if (result != sequences->num) return(E_WRITING_FILE);

	    if(sequences->formats & SEQ_REF_POS){
	    	result = fwrite(sequences->h_refPosition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_WRITING_FILE);
	    }
		if(sequences->formats & SEQ_PEQ_4x32b){
			result = fwrite(&sequences->numPEQEntries, sizeof(uint32_t), 1, fp);
	    	if (result != 1) return(E_WRITING_FILE);
	    	result = fwrite(sequences->h_PEQ, sizeof(PEQEntry_t), sequences->numPEQEntries, fp);
	    	if (result != sequences->numPEQEntries) return(E_WRITING_FILE);
	    	result = fwrite(sequences->h_PEQposition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_WRITING_FILE);
		}
		if(sequences->formats & SEQ_RAW){
			result = fwrite(&sequences->numRAWEntries, sizeof(uint32_t), 1, fp);
	    	if (result != 1) return(E_WRITING_FILE);
	    	result = fwrite(sequences->h_RAW, sizeof(RAWEntry_t), sequences->numRAWEntries, fp);
	    	if (result != sequences->numRAWEntries) return(E_WRITING_FILE);
	    	result = fwrite(sequences->h_RAWposition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_WRITING_FILE);
		}
		if(sequences->formats & SEQ_HLF_RAW){
			result = fwrite(&sequences->numRAWHlfEntries, sizeof(uint32_t), 1, fp);
	    	if (result != 1) return(E_WRITING_FILE);
	    	result = fwrite(sequences->h_HlfRAW, sizeof(RAWHlfEntry_t), sequences->numRAWHlfEntries, fp);
	    	if (result != sequences->numRAWHlfEntries) return(E_WRITING_FILE);
	    	result = fwrite(sequences->h_HlfRAWposition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_WRITING_FILE);
		}
		if(sequences->formats & SEQ_ASCII){
			result = fwrite(&sequences->numASCIIEntries, sizeof(uint32_t), 1, fp);
	    	if (result != 1) return(E_WRITING_FILE);
	    	result = fwrite(sequences->h_ASCII, sizeof(ASCIIEntry_t), sequences->numASCIIEntries, fp);
	    	if (result != sequences->numASCIIEntries) return(E_WRITING_FILE);
	    	result = fwrite(sequences->h_ASCIIposition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_WRITING_FILE);
		}
	}

	return (SUCCESS);
}

psaError_t loadSequences(FILE *fp, sequences_t *sequences)
{
	uint32_t result;

	if(sequences->formats == SEQ_VOID){

		result = fread(&sequences->num, sizeof(uint32_t), 1, fp);
        if (result != 1) return(E_READING_FILE);
    	result = fread(&sequences->formats, sizeof(seq_format_t), 1, fp);
        if (result != 1) return(E_READING_FILE);

		sequences->h_size = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
		if (sequences->h_size == NULL) return (E_ALLOCATE_MEM);
        result = fread(sequences->h_size, sizeof(uint32_t), sequences->num, fp);
	    if (result != sequences->num) return(E_READING_FILE);

	    if(sequences->formats & SEQ_REF_POS){
			sequences->h_refPosition = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
			if (sequences->h_refPosition == NULL) return (E_ALLOCATE_MEM);
	    	result = fread(sequences->h_refPosition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_WRITING_FILE);
	    }
		if(sequences->formats & SEQ_PEQ_4x32b){
			result = fread(&sequences->numPEQEntries, sizeof(uint32_t), 1, fp);
	    	if (result != 1) return(E_READING_FILE);

	    	sequences->h_PEQ = (PEQEntry_t *) malloc(sequences->numPEQEntries * sizeof(PEQEntry_t));
			if (sequences->h_PEQ == NULL) return (E_ALLOCATE_MEM);
	    	result = fread(sequences->h_PEQ, sizeof(PEQEntry_t), sequences->numPEQEntries, fp);
	    	if (result != sequences->numPEQEntries) return(E_READING_FILE);
	    	
	    	sequences->h_PEQposition = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
			if (sequences->h_PEQposition == NULL) return (E_ALLOCATE_MEM);
	    	result = fread(sequences->h_PEQposition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_READING_FILE);
		}
		if(sequences->formats & SEQ_RAW){
			result = fread(&sequences->numRAWEntries, sizeof(uint32_t), 1, fp);
	    	if (result != 1) return(E_READING_FILE);

	    	sequences->h_RAW = (RAWEntry_t *) malloc(sequences->numRAWEntries * sizeof(RAWEntry_t));
			if (sequences->h_RAW == NULL) return (E_ALLOCATE_MEM);
	    	result = fread(sequences->h_RAW, sizeof(RAWEntry_t), sequences->numRAWEntries, fp);
	    	if (result != sequences->numRAWEntries) return(E_READING_FILE);
	    	
	    	sequences->h_RAWposition = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
			if (sequences->h_RAWposition == NULL) return (E_ALLOCATE_MEM);
	    	result = fread(sequences->h_RAWposition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_READING_FILE);
		}
		if(sequences->formats & SEQ_HLF_RAW){
			result = fread(&sequences->numRAWHlfEntries, sizeof(uint32_t), 1, fp);
	    	if (result != 1) return(E_READING_FILE);

	    	sequences->h_HlfRAW = (RAWHlfEntry_t *) malloc(sequences->numRAWHlfEntries * sizeof(RAWHlfEntry_t));
			if (sequences->h_HlfRAW == NULL) return (E_ALLOCATE_MEM);
	    	result = fread(sequences->h_HlfRAW, sizeof(RAWHlfEntry_t), sequences->numRAWHlfEntries, fp);
	    	if (result != sequences->numRAWHlfEntries) return(E_READING_FILE);

	    	sequences->h_HlfRAWposition = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
			if (sequences->h_HlfRAWposition == NULL) return (E_ALLOCATE_MEM);
	    	result = fread(sequences->h_HlfRAWposition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_READING_FILE);
		}
		if(sequences->formats & SEQ_ASCII){
			result = fread(&sequences->numASCIIEntries, sizeof(uint32_t), 1, fp);
	    	if (result != 1) return(E_READING_FILE);

	    	sequences->h_ASCII = (ASCIIEntry_t *) malloc(sequences->numASCIIEntries * sizeof(ASCIIEntry_t));
			if (sequences->h_ASCII == NULL) return (E_ALLOCATE_MEM);
	    	result = fread(sequences->h_ASCII, sizeof(ASCIIEntry_t), sequences->numASCIIEntries, fp);
	    	if (result != sequences->numASCIIEntries) return(E_READING_FILE);
	    	
	    	sequences->h_ASCIIposition = (uint32_t *) malloc(sequences->num * sizeof(uint32_t));
			if (sequences->h_ASCIIposition == NULL) return (E_ALLOCATE_MEM);
	    	result = fread(sequences->h_ASCIIposition, sizeof(uint32_t), sequences->num, fp);
	    	if (result != sequences->num) return(E_READING_FILE);
		}
	}

	return (SUCCESS);
}

psaError_t freeSequences(sequences_t **sequences)
{   
	if((* sequences) != NULL){
    	if((* sequences)->h_PEQ != NULL){
        	free((* sequences)->h_PEQ);
        	(* sequences)->h_PEQ = NULL;
    	}

    	if((* sequences)->h_PEQposition != NULL){
        	free((* sequences)->h_PEQposition);
        	(* sequences)->h_PEQposition = NULL;
    	}

    	if((* sequences)->h_RAW != NULL){
        	free((* sequences)->h_RAW);
        	(* sequences)->h_RAW = NULL;
    	}

    	if((* sequences)->h_RAWposition != NULL){
        	free((* sequences)->h_RAWposition);
        	(* sequences)->h_RAWposition = NULL;
    	}

    	if((* sequences)->h_HlfRAW != NULL){
        	free((* sequences)->h_HlfRAW);
        	(* sequences)->h_HlfRAW = NULL;
    	}

    	if((* sequences)->h_HlfRAWposition != NULL){
        	free((* sequences)->h_HlfRAWposition);
        	(* sequences)->h_HlfRAWposition = NULL;
    	}

    	if((* sequences)->h_ASCII != NULL){
        	free((* sequences)->h_ASCII);
        	(* sequences)->h_ASCII = NULL;
    	}

    	if((* sequences)->h_ASCIIposition != NULL){
        	free((* sequences)->h_ASCIIposition);
        	(* sequences)->h_ASCIIposition = NULL;
    	}

    	if((* sequences)->h_size != NULL){
        	free((* sequences)->h_size);
        	(* sequences)->h_size = NULL;
    	}

    	free((* sequences));
    	(* sequences) = NULL;
    }
	
	return (SUCCESS);
}
