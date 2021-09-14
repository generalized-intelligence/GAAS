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
#include <nvbio/basic/transform_iterator.h>

namespace nvbio {

///@addtogroup Strings
///@{

///\defgroup AlphabetsModule Alphabets
///\par
/// This module provides various operators to work with the following alphabets:
///\par
/// <table>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">DNA</td>
/// <td style="vertical-align:text-top;">4-letter DNA alphabet</td>
/// <td style="vertical-align:text-top;">A,C,G,T</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">DNA_N</td>
/// <td style="vertical-align:text-top;">5-letter DNA + N alphabet</td>
/// <td style="vertical-align:text-top;">A,C,G,T,N</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">DNA_IUPAC</td>
/// <td style="vertical-align:text-top;">16-letter DNA IUPAC alphabet</td>
/// <td style="vertical-align:text-top;">=,A,C,M,G,R,S,V,T,W,Y,H,K,D,B,N</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">PROTEIN</td>
/// <td style="vertical-align:text-top;">24-letter Protein alphabet</td>
/// <td style="vertical-align:text-top;">A,C,D,E,F,G,H,I,K,L,M,N,O,P,Q,R,S,T,V,W,Y,B,Z,X</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">RNA</td>
/// <td style="vertical-align:text-top;">4-letter RNA alphabet</td>
/// <td style="vertical-align:text-top;">A,C,G,U</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">RNA_N</td>
/// <td style="vertical-align:text-top;">5-letter RNA + N alphabet</td>
/// <td style="vertical-align:text-top;">A,C,G,U,N</td></tr>
/// </table>
///
///@{

///
/// The supported sequence alphabet types
///
enum Alphabet
{
    DNA       = 0u,           ///< 4-letter DNA alphabet        { A,C,G,T }
    DNA_N     = 1u,           ///< 5-letter DNA + N alphabet    { A,C,G,T,N }
    DNA_IUPAC = 2u,           ///< 16-letter DNA IUPAC alphabet { =,A,C,M,G,R,S,V,T,W,Y,H,K,D,B,N }
    PROTEIN   = 3u,           ///< 24-letter Protein alphabet   { A,C,D,E,F,G,H,I,K,L,M,N,O,P,Q,R,S,T,V,W,Y,B,Z,X }
    RNA       = 4u,           ///< 4-letter RNA alphabet        { A,C,G,U }
    RNA_N     = 5u,           ///< 5-letter RNA + N alphabet    { A,C,G,U,N }
    ASCII     = 6u            ///< standard ASCII alphabet
};

/// A traits class for Alphabet
///
template <Alphabet ALPHABET> struct AlphabetTraits {};

/// A traits class for DNA Alphabet
///
template <> struct AlphabetTraits<DNA>
{
    static const uint32 SYMBOL_SIZE  = 2;
    static const uint32 SYMBOL_COUNT = 4;

    enum {
        A = 0,
        C = 1,
        G = 2,
        T = 3,
    };
};
/// A traits class for DNA_N Alphabet
///
template <> struct AlphabetTraits<DNA_N>
{
    static const uint32 SYMBOL_SIZE  = 4;
    static const uint32 SYMBOL_COUNT = 5;

    enum {
        A = 0,
        C = 1,
        G = 2,
        T = 3,
        N = 4,
    };
};
/// A traits class for DNA_IUPAC Alphabet
///
template <> struct AlphabetTraits<DNA_IUPAC>
{
    static const uint32 SYMBOL_SIZE  = 4;
    static const uint32 SYMBOL_COUNT = 16;

    enum {
        EQUAL = 0,
        A     = 1,
        C     = 2,
        M     = 3,
        G     = 4,
        R     = 5,
        S     = 6,
        V     = 7,
        T     = 8,
        W     = 9,
        Y     = 10,
        H     = 11,
        K     = 12,
        D     = 13,
        B     = 14,
        N     = 15,
    };
};
/// A traits class for Protein Alphabet
///
template <> struct AlphabetTraits<PROTEIN>
{
    static const uint32 SYMBOL_SIZE  = 8;
    static const uint32 SYMBOL_COUNT = 24;

    enum {
        A = 0,
        C = 1,
        D = 2,
        E = 3,
        F = 4,
        G = 5,
        H = 6,
        I = 7,
        K = 8,
        L = 9,
        M = 10,
        N = 11,
        O = 12,
        P = 13,
        Q = 14,
        R = 15,
        S = 16,
        T = 17,
        V = 18,
        W = 19,
        Y = 20,
        B = 21,
        Z = 22,
        X = 23,
    };
};
/// A traits class for DNA Alphabet
///
template <> struct AlphabetTraits<RNA>
{
    static const uint32 SYMBOL_SIZE  = 2;
    static const uint32 SYMBOL_COUNT = 4;

    enum {
        A = 0,
        C = 1,
        G = 2,
        U = 3,
    };
};
/// A traits class for DNA_N Alphabet
///
template <> struct AlphabetTraits<RNA_N>
{
    static const uint32 SYMBOL_SIZE  = 4;
    static const uint32 SYMBOL_COUNT = 5;

    enum {
         A = 0,
         C = 1,
         G = 2,
         U = 3,
         N = 4,
    };
};
/// A traits class for ASCII Alphabet
///
template <> struct AlphabetTraits<ASCII>
{
    static const uint32 SYMBOL_SIZE  = 8;
    static const uint32 SYMBOL_COUNT = 256;
};

/// return the number of bits per symbol for a given alphabet
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 bits_per_symbol(const Alphabet alphabet)
{
    return alphabet == DNA       ? 2 :
           alphabet == DNA_N     ? 4 :
           alphabet == DNA_IUPAC ? 4 :
           alphabet == PROTEIN   ? 8 :
           alphabet == RNA       ? 2 :
           alphabet == RNA_N     ? 4 :
           alphabet == ASCII     ? 8 :
           8u;
}

/// convert a given symbol to its ASCII character
///
template <Alphabet ALPHABET>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE char to_char(const uint8 c);

/// convert a given symbol to its ASCII character
///
template <Alphabet ALPHABET>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint8 from_char(const char c);

/// convert from the given alphabet to an ASCII string
///
template <Alphabet ALPHABET, typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void to_string(
    const SymbolIterator begin,
    const uint32         n,
    char*                string);

/// convert from the given alphabet to an ASCII string
///
template <Alphabet ALPHABET, typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void to_string(
    const SymbolIterator begin,
    const SymbolIterator end,
    char*                string);

/// convert from an ASCII string to the given alphabet
///
template <Alphabet ALPHABET, typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void from_string(
    const char*             begin,
    const char*             end,
    SymbolIterator          symbols);

/// convert from an ASCII string to the given alphabet
///
template <Alphabet ALPHABET, typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void from_string(
    const char*             begin,
    SymbolIterator          symbols);

/// conversion functor from a given alphabet to ASCII char
///
template <Alphabet ALPHABET>
struct to_char_functor
{
    typedef uint8 argument_type;
    typedef char  result_type;

    /// functor operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE char operator() (const uint8 c) const { return to_char<ALPHABET>( c ); }
};

/// conversion functor from a given alphabet to ASCII char
///
template <Alphabet ALPHABET>
struct from_char_functor
{
    typedef char  argument_type;
    typedef uint8 result_type;

    /// functor operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint8 operator() (const char c) const { return from_char<ALPHABET>( c ); }
};

/// convert a string iterator from a given alphabet to an ASCII string iterator
///
template <Alphabet ALPHABET, typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
transform_iterator< Iterator, to_char_functor<ALPHABET> > 
to_string(Iterator it)
{
    return make_transform_iterator( it, to_char_functor<ALPHABET>() );
}

/// convert an ASCII string iterator to a given alphabet string iterator
///
template <Alphabet ALPHABET, typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
transform_iterator< Iterator, from_char_functor<ALPHABET> >
from_string(Iterator it)
{
    return make_transform_iterator( it, from_char_functor<ALPHABET>() );
}

///@} AlphabetsModule
///@} Strings

} // namespace nvbio

#include <nvbio/strings/alphabet_inl.h>
