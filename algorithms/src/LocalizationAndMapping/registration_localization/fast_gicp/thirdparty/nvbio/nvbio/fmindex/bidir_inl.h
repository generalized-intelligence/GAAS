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

namespace nvbio {

// \relates fm_index
// forward extension using a bidirectional FM-index, extending the range
// of a pattern P to the pattern Pc
//
// \param f_fmi    forward FM-index
// \param r_fmi    reverse FM-index
// \param f_range  current forward range
// \param r_range  current reverse range
// \param c        query character
//
template <
    typename TRankDictionary1,
    typename TSuffixArray1,
    typename TRankDictionary2,
    typename TSuffixArray2>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void extend_forward(
    const fm_index<TRankDictionary1,TSuffixArray1>&                 f_fmi,
    const fm_index<TRankDictionary2,TSuffixArray2>&                 r_fmi,
    typename fm_index<TRankDictionary1,TSuffixArray1>::range_type&  f_range,
    typename fm_index<TRankDictionary2,TSuffixArray2>::range_type&  r_range,
    uint8                                                           c)
{
    typedef typename fm_index<TRankDictionary1,TSuffixArray1>::range_type f_range_type;
    typedef typename fm_index<TRankDictionary2,TSuffixArray2>::range_type r_range_type;

    // find the number of suffixes in T that start with Pd, for d < c
    uint32 x = 0;
    for (uint32 d = 0; d < c; ++d)
    {
        // search for (Pd)^R = dP^R in r_fmi
        const r_range_type d_rank = rank(
            r_fmi,
            make_vector( r_range.x-1, r_range.y ),
            d );

        // add the number of occurrences to x
        x += d_rank.y - d_rank.x;
    }

    // search for (Pc)^R = cP^R in r_fmi
    {
        const r_range_type c_rank = rank(
            r_fmi,
            make_vector( r_range.x-1, r_range.y ),
            c );

        r_range.x = r_fmi.L2(c) + c_rank.x + 1;
        r_range.y = r_fmi.L2(c) + c_rank.y;
    }
    const uint32 y = 1u + r_range.y - r_range.x;

    // and now compute the new forward range of Pc
    f_range.y = f_range.x + x + y - 1u;
    f_range.x = f_range.x + x;
}

// \relates fm_index
// backwards extension using a bidirectional FM-index, extending the range
// of a pattern P to the pattern cP
//
// \param f_fmi    forward FM-index
// \param r_fmi    reverse FM-index
// \param f_range  current forward range
// \param r_range  current reverse range
// \param c        query character
//
template <
    typename TRankDictionary1,
    typename TSuffixArray1,
    typename TRankDictionary2,
    typename TSuffixArray2>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void extend_backwards(
    const fm_index<TRankDictionary1,TSuffixArray1>&                 f_fmi,
    const fm_index<TRankDictionary2,TSuffixArray2>&                 r_fmi,
    typename fm_index<TRankDictionary1,TSuffixArray1>::range_type&  f_range,
    typename fm_index<TRankDictionary2,TSuffixArray2>::range_type&  r_range,
    uint8                                                           c)
{
    typedef typename fm_index<TRankDictionary1,TSuffixArray1>::range_type f_range_type;
    typedef typename fm_index<TRankDictionary2,TSuffixArray2>::range_type r_range_type;

    // find the number of suffixes in T that start with dP, for d < c
    uint32 x = 0;
    for (uint32 d = 0; d < c; ++d)
    {
        // search for dP in f_fmi
        const f_range_type d_rank = rank(
            f_fmi,
            make_vector( f_range.x-1, f_range.y ),
            d );

        // add the number of occurrences to x
        x += d_rank.y - d_rank.x;
    }

    // search for cP in f_fmi
    {
        const f_range_type c_rank = rank(
            f_fmi,
            make_vector( f_range.x-1, f_range.y ),
            c );

        f_range.x = f_fmi.L2(c) + c_rank.x + 1;
        f_range.y = f_fmi.L2(c) + c_rank.y;
    }
    const uint32 y = 1u + f_range.y - f_range.x;

    // and now compute the new reverse range of cP
    r_range.y = r_range.x + x + y - 1u;
    r_range.x = r_range.x + x;
}

} // namespace nvbio
