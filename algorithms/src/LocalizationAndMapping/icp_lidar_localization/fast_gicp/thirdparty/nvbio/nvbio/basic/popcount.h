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

///@addtogroup Basic
///@{

///@addtogroup BasicUtils Utilities
///@{

/// int32 popcount
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc(const int32 i);

/// uint32 popcount
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc(const uint32 i);

/// uint8 popcount
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc(const uint8 i);

/// find the n-th bit set in a 4-bit mask (n in [1,4])
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 find_nthbit4(const uint32 mask, const uint32 n);

/// compute the pop-count of 4-bit mask
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc4(const uint32 mask);

/// find the n-th bit set in a 8-bit mask (n in [1,8])
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 find_nthbit8(const uint32 mask, const uint32 n);

/// find the least significant bit set
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 ffs(const int32 x);


/// count the number of leading zeros
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 lzc(const uint32 x);


/// count the number of occurrences of a given 2-bit pattern in a given word
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit(const uint32 x, int c);

/// count the number of occurrences of all 2-bit patterns in a given word,
/// using an auxiliary table.
///
/// \param count_table      auxiliary table to perform the parallel bit-counting
///                         for all integers in the range [0,255].
///
/// \return                 the 4 pop counts shifted and OR'ed together
///
template <typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit_all(
    const uint32 b,
    const CountTable count_table);

/// given a 32-bit word encoding a set of 2-bit symbols, return a submask containing
/// all but the first 'i' entries.
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 hibits_2bit(const uint32 mask, const uint32 i);

/// count the number of occurrences of a given 2-bit pattern in all but the first 'i' symbols
/// of a 32-bit word mask.
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit(const uint32 mask, int c, const uint32 i);

/// count the number of occurrences of a given N-bit pattern in a given word
///
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_nbit(const uint32 x, int c);

/// given a 32-bit word encoding a set of N-bit symbols, return a submask containing
/// all but the first 'i' entries.
///
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 hibits_nbit(const uint32 mask, const uint32 i);

/// count the number of occurrences of a given N-bit pattern in all but the first 'i' symbols
/// of a 32-bit word mask.
///
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_nbit(const uint32 mask, int c, const uint32 i);

/// count the number of occurrences of all 2-bit patterns in all but the first 'i' symbols
/// of a given word, using an auxiliary table.
///
/// \param count_table      auxiliary table to perform the parallel bit-counting
///                         for all integers in the range [0,255].
///
/// \return                 the 4 pop counts shifted and OR'ed together
///
template <typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit_all(
    const uint32     mask,
    const CountTable count_table,
    const uint32     i);

// generate table for counting 11,10,01,00(pattern) for 8 bits number
// table [no# ] = representation ( # of count-pattern, . , . , . )
// ---------------------------------------------------------------------------
// e.g cnt_table[11111111] = 0x04000000 ( 4-11, 0-10, 0-01, 0-00 )
// cnt_table[00100001] = 0x00010102 ( 0-11, 1-10, 1-01, 2-00 )
// cnt_table[00000001] = 0x00000103 ( 0-11, 0-10, 1-01, 3-00 )
inline void gen_2bit_count_table(uint32* count_table)
{
    for (int i = 0; i != 256; ++i)
    {
        uint32 x = 0;
        for (int j = 0; j != 4; ++j)
            x |= (((i&3) == j) + ((i>>2&3) == j) + ((i>>4&3) == j) + (i>>6 == j)) << (j<<3);

        count_table[i] = x;
    }
}

///@} BasicUtils
///@} Basic

} // namespace nvbio

#include <nvbio/basic/popcount_inl.h>
