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
#include <sais.h>

namespace nvbio {

/// helper function to generate a suffix array padded by 1, where
/// the 0-th entry is the SA size.
///
template <typename StreamIterator>
uint32 gen_sa(const uint32 n, const StreamIterator T, int32 *SA)
{
  SA[0] = n;
  if (n <= 1) {
      if (n == 1) SA[1] = 0;
      return 0;
  }
  return saisxx( T, SA+1, int32(n), 4 );
}

/// helper function to generate the BWT of a string given its suffix array.
///
template <typename StreamIterator>
uint32 gen_bwt_from_sa(const uint32 n, const StreamIterator T, const int32* SA, StreamIterator bwt)
{
    uint32 i, primary = 0;

    for (i = 0; i <= n; ++i)
    {
        if (SA[i] == 0) primary = i;
        else bwt[i] = T[SA[i] - 1];
    }
    for (i = primary; i < n; ++i) bwt[i] = bwt[i + 1];
    return primary;
}

/// helper function to generate the BWT of a string given a temporary buffer.
///
template <typename StreamIterator>
int32 gen_bwt(const uint32 n, const StreamIterator T, int32* buffer, StreamIterator bwt)
{
    return saisxx_bwt( T, bwt, buffer, int32(n), 4 );
}

/// helper function to generate the BWT of a string given a temporary buffer.
///
template <typename StreamIterator>
int64 gen_bwt(const uint32 n, const StreamIterator T, int64* buffer, StreamIterator bwt)
{
    return saisxx_bwt( T, bwt, buffer, int64(n), int64(4) );
}

// generate table for counting 11,10,01,00(pattern) for 8 bits number
// table [no# ] = representation ( # of count-pattern, . , . , . )
// ---------------------------------------------------------------------------
// e.g cnt_table[11111111] = 0x04000000 ( 4-11, 0-10, 0-01, 0-00 )
// cnt_table[00100001] = 0x00010102 ( 0-11, 1-10, 1-01, 2-00 )
// cnt_table[00000001] = 0x00000103 ( 0-11, 0-10, 1-01, 3-00 )
inline void gen_bwt_count_table(uint32* count_table)
{
    for (int i = 0; i != 256; ++i)
    {
        uint32 x = 0;
        for (int j = 0; j != 4; ++j)
            x |= (((i&3) == j) + ((i>>2&3) == j) + ((i>>4&3) == j) + (i>>6 == j)) << (j<<3);

        count_table[i] = x;
    }
}

} // namespace nvbio

