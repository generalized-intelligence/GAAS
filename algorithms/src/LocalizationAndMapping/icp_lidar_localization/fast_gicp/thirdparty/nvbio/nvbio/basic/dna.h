/*
 * nvbio
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the NVIDIA CORPORATION nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <nvbio/basic/types.h>

namespace nvbio {

/// convert a 2-bit DNA symbol to its ASCII character
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE char dna_to_char(const uint8 c)
{
    return c == 0 ? 'A' :
           c == 1 ? 'C' :
           c == 2 ? 'G' :
           c == 3 ? 'T' :
                    'N';
}

/// convert a 2-bit DNA symbol to a IUPAC16 symbol
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint8 dna_to_iupac16(const uint8 c)
{
    //     DNA: A, C, G, T -> { 0, 1, 2, 3 }
    // IUPAC16: A, C, G, T -> { 1, 2, 4, 8 }
    return 1 << c;
}

/// convert a 4-bit DNA symbol to its ASCII character
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE char iupac16_to_char(const uint8 c)
{
    return c ==  0 ? '=' :
           c ==  1 ? 'A' :
           c ==  2 ? 'C' :
           c ==  3 ? 'M' :
           c ==  4 ? 'G' :
           c ==  5 ? 'R' :
           c ==  6 ? 'S' :
           c ==  7 ? 'V' :
           c ==  8 ? 'T' :
           c ==  9 ? 'W' :
           c == 10 ? 'Y' :
           c == 11 ? 'H' :
           c == 12 ? 'K' :
           c == 13 ? 'D' :
           c == 14 ? 'B' :
                     'N';
}

/// convert an ASCII DNA representation to its 2-bit symbol
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint8 char_to_dna(const char c)
{
    return c == 'A' ? 0u :
           c == 'C' ? 1u :
           c == 'G' ? 2u :
           c == 'T' ? 3u :
                      4u;
}

/// convert an ASCII DNA representation to its 4-bit symbol
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint8 char_to_iupac16(const char c)
{
    return c == '=' ?  0u :
           c == 'A' ?  1u :
           c == 'C' ?  2u :
           c == 'M' ?  3u :
           c == 'G' ?  4u :
           c == 'R' ?  5u :
           c == 'S' ?  6u :
           c == 'V' ?  7u :
           c == 'T' ?  8u :
           c == 'W' ?  9u :
           c == 'Y' ? 10u :
           c == 'H' ? 11u :
           c == 'K' ? 12u :
           c == 'D' ? 13u :
           c == 'B' ? 14u :
                      15u;
}

/// convert a 2-bit DNA string to an ASCII string
///
template <typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void dna_to_string(
    const SymbolIterator begin,
    const uint32 n,
    char* string)
{
    for (uint32 i = 0; i < n; ++i)
        string[i] = dna_to_char( begin[i] );

    string[n] = '\0';
}

/// convert a 2-bit DNA string to an ASCII string
///
template <typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void dna_to_string(
    const SymbolIterator begin,
    const SymbolIterator end,
    char* string)
{
    for (SymbolIterator it = begin; it != end; ++it)
        string[ (it - begin) % (end - begin) ] = dna_to_char( *it );

    string[ end - begin ] = '\0';
}

/// convert a 2-bit DNA string to an ASCII string
///
template <typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void iupac16_to_string(
    const SymbolIterator begin,
    const uint32 n,
    char* string)
{
    for (uint32 i = 0; i < n; ++i)
        string[i] = iupac16_to_char( begin[i] );

    string[n] = '\0';
}

/// convert a 2-bit DNA string to an ASCII string
///
template <typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void iupac16_to_string(
    const SymbolIterator begin,
    const SymbolIterator end,
    char* string)
{
    for (SymbolIterator it = begin; it != end; ++it)
        string[ (it - begin) % (end - begin) ] = iupac16_to_char( *it );

    string[ end - begin ] = '\0';
}

/// convert an ASCII DNA string to its 2-bit representation
///
template <typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void string_to_dna(
    const char* begin,
    const char* end,
    SymbolIterator symbols)
{
    for (const char* it = begin; it != end; ++it)
        symbols[ (it - begin) % (end - begin) ] = char_to_dna( *it );
}

/// convert a NULL-terminated ASCII DNA string to its 2-bit representation
///
template <typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void string_to_dna(
    const char* begin,
    SymbolIterator symbols)
{
    for (const char* it = begin; *it != '\0'; ++it)
        symbols[ it - begin ] = char_to_dna( *it );
}

/// convert an ASCII DNA string to its 4-bit representation
///
template <typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void string_to_iupac16(
    const char* begin,
    const char* end,
    SymbolIterator symbols)
{
    for (const char* it = begin; it != end; ++it)
        symbols[ (it - begin) % (end - begin) ] = char_to_iupac16( *it );
}

/// convert a NULL-terminated ASCII DNA string to its 4-bit representation
///
template <typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void string_to_iupac16(
    const char* begin,
    SymbolIterator symbols)
{
    for (const char* it = begin; *it != '\0'; ++it)
        symbols[ it - begin ] = char_to_iupac16( *it );
}

} // namespace nvbio

