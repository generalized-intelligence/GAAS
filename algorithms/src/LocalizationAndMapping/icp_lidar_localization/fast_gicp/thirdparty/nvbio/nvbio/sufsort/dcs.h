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

#include <nvbio/sufsort/sufsort_priv.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/basic/thrust_view.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/counting_iterator.h>

namespace nvbio {


// Precomputed Difference Covers
template <uint32 Q> struct DCTable {};

// Precomputed DC-64
template <> struct DCTable<64>
{
    static const uint32 N = 9;          // DC quorum

    static const uint32* S()
    {
        static const uint32 dc[9] = { 1, 2, 3, 6, 15, 17, 35, 43, 60 };
        return dc;
    }
};
// Precomputed DC-128
template <> struct DCTable<128>
{
    static const uint32 N = 16;         // DC quorum

    static const uint32* S()
    {
        static const uint32 dc[16] = { 0, 1, 2, 5, 10, 15, 26, 37, 48, 59, 70, 76, 82, 88, 89, 90 };
        return dc;
    }
};
// Precomputed DC-256
template <> struct DCTable<256>
{
    static const uint32 N = 22;         // DC quorum

    static const uint32* S()
    {
        static const uint32 dc[22] = { 0, 1, 2, 3, 7, 14, 21, 28, 43, 58, 73, 88, 103, 118, 133, 141, 149, 157, 165, 166, 167, 168 };
        return dc;
    }
};
// Precomputed DC-512
template <> struct DCTable<512>
{
    static const uint32 N = 28;         // DC quorum

    static const uint32* S()
    {
        static const uint32 dc[28] = { 0, 1, 2, 3, 4, 9, 18, 27, 36, 45, 64, 83, 102, 121, 140, 159, 178, 197, 216, 226, 236, 246, 256, 266, 267, 268, 269, 270 };
        return dc;
    }
};
// Precomputed DC-1024
template <> struct DCTable<1024>
{
    static const uint32 N = 40;         // DC quorum

    static const uint32* S()
    {
        static const uint32 dc[40] = { 0, 1, 2, 3, 4, 5, 6, 13, 26, 39, 52, 65, 78, 91, 118, 145, 172, 199, 226, 253, 280, 307, 334, 361, 388, 415, 442, 456, 470, 484, 498, 512, 526, 540, 541, 542, 543, 544, 545, 546 };
        return dc;
    }
};
// Precomputed DC-2048
template <> struct DCTable<2048>
{
    static const uint32 N = 58;         // DC quorum

    static const uint32* S()
    {
        static const uint32 dc[58] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 19, 38, 57, 76, 95, 114, 133, 152, 171, 190, 229, 268, 307, 346, 385, 424, 463, 502, 541, 580, 619, 658, 697, 736, 775, 814, 853, 892, 931, 951, 971, 991, 1011, 1031, 1051, 1071, 1091, 1111, 1131, 1132, 1133, 1134, 1135, 1136, 1137, 1138, 1139, 1140 };
        return dc;
    }
};

/// A data structure to hold a Difference Cover Sample
///
struct DCSView
{
    DCSView(
        const uint32  _Q        = 0,
        const uint32  _N        = 0,
        const uint32  _size     = 0,
              uint32* _dc       = NULL,
              uint32* _lut      = NULL,
              uint32* _pos      = NULL,
              uint8*  _bitmask  = NULL,
              uint32* _ranks    = NULL) :
        Q       ( _Q ),
        N       ( _N ),
        dc      ( _dc ),
        lut     ( _lut ),
        pos     ( _pos ),
        bitmask ( _bitmask ),
        ranks   ( _ranks ),
        size    ( _size ) {}


    /// return the sampled position of a given suffix index
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 index(const uint32 i) const;

    const uint32    Q;
    const uint32    N;
    uint32*         dc;
    uint32*         lut;
    uint32*         pos;
    uint8*          bitmask;
    uint32*         ranks;
    const uint32    size;
};

/// A data structure to hold a Difference Cover Sample
///
struct DCS
{
    typedef DCSView plain_view_type;

    /// constructor
    ///
    template <uint32 QT>
    void init();

    /// estimate sample size
    ///
    template <uint32 QT>
    static uint32 estimated_sample_size(const uint64 string_len) { return uint32( util::divide_ri( string_len * DCTable<QT>::N, QT ) + 1u ); }

    /// estimate sample size
    ///
    uint32 estimate_sample_size(const uint64 string_len) const { return uint32( util::divide_ri( string_len * N, Q ) + 1u ); }

    uint32                        Q;            ///< difference cover period
    uint32                        N;            ///< difference cover quorum

    thrust::device_vector<uint32> d_dc;         ///< difference cover table
    thrust::device_vector<uint32> d_lut;        ///< the (i,j) -> l LUT
    thrust::device_vector<uint32> d_pos;        ///< the DC -> pos mapping
    thrust::device_vector<uint8>  d_bitmask;    ///< difference cover bitmask
    thrust::device_vector<uint32> d_ranks;      ///< ordered DCS ranks
};

/// return the plain view of a DCS
///
inline DCSView plain_view(DCS& dcs)
{
    return DCSView(
        dcs.Q,
        dcs.N,
        uint32( dcs.d_ranks.size() ),
        nvbio::plain_view( dcs.d_dc ),
        nvbio::plain_view( dcs.d_lut ),
        nvbio::plain_view( dcs.d_pos ),
        nvbio::plain_view( dcs.d_bitmask ),
        nvbio::plain_view( dcs.d_ranks ) );
}

/// return the plain view of a DCS
///
inline DCSView plain_view(const DCS& dcs)
{
    return plain_view( const_cast<DCS&>( dcs ) );
}

namespace priv {

/// A functor to evaluate whether an index is in a Difference Cover Sample
///
struct DCS_predicate
{
    typedef uint32 argument_type;
    typedef uint32 result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    DCS_predicate(const uint32 _Q, const uint8* _dc_bitmask) : Q(_Q), dc_bitmask(_dc_bitmask) {}

    /// return whether the given integer is the DC
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint32 suffix) const { return (dc_bitmask[ suffix & (Q-1) ] != 0); }

    const uint32 Q;
    const uint8* dc_bitmask;
};

/// A functor to transform a global suffix index into a DCS-local index
///
struct DCS_string_suffix_index
{
    typedef uint32   argument_type;
    typedef uint32   result_type;

    DCS_string_suffix_index(const DCSView _dcs) : dcs( _dcs ) {}

    /// return true if the first suffix is lexicographically smaller than the second, false otherwise
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint32 suffix_idx) const
    {
        return nvbio::min( dcs.index( suffix_idx ), dcs.size );
    }

    const DCSView dcs;
};

/// A binary functor comparing two suffixes lexicographically using a Difference Cover Sample
///
template <uint32 SYMBOL_SIZE, typename string_type>
struct DCS_string_suffix_less
{
    typedef uint32   first_argument_type;
    typedef uint32   second_argument_type;
    typedef uint32   result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    DCS_string_suffix_less(
        const uint64        _string_len,
        const string_type   _string,
        const DCSView       _dcs) :
        string_len(_string_len),
        string(_string),
        dcs( _dcs ) {}

    /// return true if the first suffix is lexicographically smaller than the second, false otherwise
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const uint64 suffix_idx1, const uint64 suffix_idx2) const
    {
        const uint32 Q = dcs.Q;

      //#define DCS_CHECKS
      #if defined(DCS_CHECKS)
        const string_suffix_less<SYMBOL_SIZE,string_type> less( string_len, string );
        const bool r  = less( suffix_idx1, suffix_idx2 );
      #endif

        const uint32 WORD_BITS   = 32u; // use 32-bit words
        const uint32 DOLLAR_BITS = 4u;  // 4 is the minimum number needed to encode up to 16 symbols per word
        const uint32 SYMBOLS_PER_WORD = symbols_per_word<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS>();

        const uint32 suffix_len1 = string_len - suffix_idx1;
        const uint32 suffix_len2 = string_len - suffix_idx2;
        const uint32 q_words = (Q + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;
        const uint32 n_words = nvbio::min(
            uint32( nvbio::min(
                suffix_len1,
                suffix_len2 ) + SYMBOLS_PER_WORD-1 ) / SYMBOLS_PER_WORD,
                q_words );

        // loop through all string-words
        for (uint32 w = 0; w < n_words; ++w)
        {
            string_suffix_word_functor<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS,string_type,uint32> word_functor( string_len, string, w );

            const uint32 w1 = word_functor( suffix_idx1 );
            const uint32 w2 = word_functor( suffix_idx2 );
            if (w1 < w2) return true;
            if (w1 > w2) return false;
        }

        // check whether the suffixes are shorter than Q - this should never happen...
        if (suffix_len1 < Q ||
            suffix_len2 < Q)
        {
            #if defined(DCS_CHECKS)
            const bool r2 = suffix_len1 < suffix_len2;
            if (r != r2)
                printf("short suffixes %u, %u\n", suffix_len1, suffix_len2 );
            #endif

            return suffix_len1 < suffix_len2;
        }

        // compare the DCS ranks
        {
            const uint32 i_mod_Q = suffix_idx1 & (Q-1);
            const uint32 j_mod_Q = suffix_idx2 & (Q-1);

            // lookup the smallest number l such that (i + l) and (j + l) are in the DCS
            const uint32 l = dcs.lut[ i_mod_Q * Q + j_mod_Q ];

            // by construction (suffix_idx1 + l) and (suffix_idx2 + l) are both in the DCS,
            // we just need to find exactly where...
            const uint32 pos_i = dcs.index( suffix_idx1 + l );
            const uint32 pos_j = dcs.index( suffix_idx2 + l );

            // now we can lookup the ranks of the suffixes in the DCS
            const uint32 rank_i = dcs.ranks[ pos_i ];
            const uint32 rank_j = dcs.ranks[ pos_j ];

            #if defined(DCS_CHECKS)
            const bool r2 = rank_i < rank_j;
            if (r != r2)
            {
                printf("(%u,%u) : %u != %u, l[%u], pos[%u,%u], rank[%u,%u]\n",
                    (uint32)suffix_idx1, (uint32)suffix_idx2,
                    (uint32)r,
                    (uint32)r2,
                    (uint32)l,
                    (uint32)pos_i,
                    (uint32)pos_j,
                    (uint32)rank_i,
                    (uint32)rank_j);
            }
            #endif

            return rank_i < rank_j;
        }
    }

    const uint64        string_len;
    const string_type   string;
    const DCSView       dcs;
};

} // namespace priv
} // namespace nvbio

#include <nvbio/sufsort/dcs_inl.h>
