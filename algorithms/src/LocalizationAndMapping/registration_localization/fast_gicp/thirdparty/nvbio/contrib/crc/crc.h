/**********************************************************************
 *
 * Filename:    crc.h
 * 
 * Description: A header file describing the various CRC standards.
 *
 * Notes:       
 *
 * 
 * Copyright (c) 2000 by Michael Barr.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 **********************************************************************/

#ifndef _crc_h
#define _crc_h

#if !defined(CRC_FALSE)
#define CRC_FALSE	0
#endif
#if !defined(CRC_TRUE)
#define CRC_TRUE	!CRC_FALSE
#endif

/*
 * Select the CRC standard from the list that follows.
 */
#define CRC32


#if defined(CRC_CCITT)

typedef unsigned short  crc;

#define CRC_NAME			"CRC-CCITT"
#define POLYNOMIAL			0x1021
#define INITIAL_REMAINDER	0xFFFF
#define FINAL_XOR_VALUE		0x0000
#define REFLECT_DATA		CRC_FALSE
#define REFLECT_REMAINDER	CRC_FALSE
#define CHECK_VALUE			0x29B1

#elif defined(CRC16)

typedef unsigned short  crc;

#define CRC_NAME			"CRC-16"
#define POLYNOMIAL			0x8005
#define INITIAL_REMAINDER	0x0000
#define FINAL_XOR_VALUE		0x0000
#define REFLECT_DATA		CRC_TRUE
#define REFLECT_REMAINDER	CRC_TRUE
#define CHECK_VALUE			0xBB3D

#elif defined(CRC32)

// xxxnsubtil: unsigned long was a 64-bit type for us! switched to nvbio's uint32 instead
// xxxjpantaleoni: removed dependeny on nvbio and switched to unsigned int
typedef unsigned int crc;

#define CRC_NAME			"CRC-32"
#define POLYNOMIAL			0x04C11DB7
#define INITIAL_REMAINDER	0xFFFFFFFF
#define FINAL_XOR_VALUE		0xFFFFFFFF
#define REFLECT_DATA		CRC_TRUE
#define REFLECT_REMAINDER	CRC_TRUE
#define CHECK_VALUE			0xCBF43926

#else

#error "One of CRC_CCITT, CRC16, or CRC32 must be #define'd."

#endif

/*
 * Derive parameters from the standard-specific parameters in crc.h.
 */
#define WIDTH    (8 * sizeof(crc))
#define TOPBIT   (1 << (WIDTH - 1))

#if (REFLECT_DATA == CRC_TRUE)
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			((unsigned char) reflect((X), 8))
#else
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			(X)
#endif

#if (REFLECT_REMAINDER == CRC_TRUE)
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	((crc) reflect((X), WIDTH))
#else
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	(X)
#endif

unsigned long reflect(unsigned long data, unsigned char nBits);

extern crc  crcTable[256];

void  crcInit(void);

/*********************************************************************
 *
 * Function:    crcCalc()
 * 
 * Description: Compute the CRC of a given message.
 *
 * Notes:		crcInit() must be called first.
 *
 * Returns:		The CRC of the message.
 *
 *********************************************************************/
template <typename CharIterator>
crc crcCalc(const CharIterator message, unsigned int nBytes)
{
    crc	           remainder = INITIAL_REMAINDER;
    unsigned char  data;
	unsigned int   byte;

    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (byte = 0; byte < nBytes; ++byte)
    {
        data = REFLECT_DATA(message[byte]) ^ (remainder >> (WIDTH - 8));
  		remainder = crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);
}

#endif /* _crc_h */