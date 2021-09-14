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

#include <nvbio/basic/packedstream.h>
#include <nvbio/alignment/sink.h>

namespace nvbio {
namespace aln {

//
// Calculate the maximum possible number of pattern gaps that could occur in a
// given score boundary
//
template <AlignmentType TYPE, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_pattern_gaps(
    const EditDistanceAligner<TYPE,algorithm_tag>&  aligner,
	int32                                           min_score,
    int32                                           pattern_len)
{
    return -min_score;
}

//
// Calculate the maximum possible number of reference gaps that could occur in a
// given score boundary
//
template <AlignmentType TYPE, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_text_gaps(
    const EditDistanceAligner<TYPE,algorithm_tag>&  aligner,
	int32                                           min_score,
    int32                                           pattern_len)
{
    return -min_score;
}

//
// Calculate the maximum possible number of pattern gaps that could occur in a
// given score boundary
//
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_pattern_gaps(
    const HammingDistanceAligner<TYPE,scoring_scheme_type,algorithm_tag>&   aligner,
	int32                                                                   min_score,
    int32                                                                   pattern_len)
{
    return 0;
}

//
// Calculate the maximum possible number of reference gaps that could occur in a
// given score boundary
//
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_text_gaps(
    const HammingDistanceAligner<TYPE,scoring_scheme_type,algorithm_tag>&   aligner,
	int32                                                                   min_score,
    int32                                                                   pattern_len)
{
    return 0;
}

//
// Calculate the maximum possible number of pattern gaps that could occur in a
// given score boundary
//
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_pattern_gaps(
    const SmithWatermanAligner<TYPE,scoring_scheme_type,algorithm_tag>& scoring,
	int32                                                               min_score,
	int32                                                               pattern_len)
{
	// compute the optimal score
	int32 score = pattern_len * scoring.scheme.match(30);
	if (score < min_score)
        return 0u;

    uint32 n = 0;
	while (score >= min_score && n < pattern_len)
    {
		// subtract just the extension penalty
		score += scoring.scheme.deletion();

		++n;
	}
	return n-1;
}

//
// Calculate the maximum possible number of reference gaps that could occur in a
// given score boundary
//
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_text_gaps(
    const SmithWatermanAligner<TYPE,scoring_scheme_type,algorithm_tag>& scoring,
	int32                                                               min_score,
	int32                                                               pattern_len)
{
	// compute the optimal score
	int32 score = pattern_len * scoring.scheme.match(30);
	if (score < min_score)
        return 0u;

    uint32 n = 0;
	while (score >= min_score && n < pattern_len)
    {
		// subtract just the extension penalty
		score += scoring.scheme.insertion();

		++n;
	}
	return n-1;
}

//
// Calculate the maximum possible number of pattern gaps that could occur in a
// given score boundary
//
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_pattern_gaps(
    const GotohAligner<TYPE,scoring_scheme_type,algorithm_tag>& scoring,
	int32                                                       min_score,
	int32                                                       pattern_len)
{
	// compute the optimal score
	int32 score = pattern_len * scoring.scheme.match(30);
	if (score < min_score)
        return 0u;

    // subtract the gap open penalty
	score += scoring.scheme.pattern_gap_open();

    uint32 n = 0;
	while (score >= min_score && n < pattern_len)
    {
		// subtract just the extension penalty
		score += scoring.scheme.pattern_gap_extension();

		++n;
	}
	return n-1;
}

//
// Calculate the maximum possible number of reference gaps that could occur in a
// given score boundary
//
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_text_gaps(
    const GotohAligner<TYPE,scoring_scheme_type,algorithm_tag>& scoring,
	int32                                                       min_score,
	int32                                                       pattern_len)
{
	// compute the optimal score
	int32 score = pattern_len * scoring.scheme.match(30);
	if (score < min_score)
        return 0u;

    // subtract the gap open penalty
	score += scoring.scheme.text_gap_open();

    uint32 n = 0;
	while (score >= min_score && n < pattern_len)
    {
		// subtract just the extension penalty
		score += scoring.scheme.text_gap_extension();

		++n;
	}
	return n-1;
}

template <uint32 BAND_LEN, typename score_type>
struct select_dispatch
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static score_type enact(const uint32 M, const score_type* band)
    {
        // get the highest score along the long edge of the path graph
        score_type r = 0;

        const uint32 M_mod = (M-1) & (BAND_LEN-1);

        #pragma unroll
        for (uint32 j = 1; j <= BAND_LEN; ++j)
        {
            const score_type value = band[j];
            if (j == M_mod+1)
                r = value;
        }
        return r;
    }
};
template <uint32 BAND_LEN>
struct select_dispatch<BAND_LEN, simd4u8>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static simd4u8 enact(const uint4 M, const simd4u8* band)
    {
        // get the highest score along the long edge of the path graph
        uint4 r;

        const uint4 M_mod = make_uint4(
            (M.x-1) & (BAND_LEN-1),
            (M.y-1) & (BAND_LEN-1),
            (M.z-1) & (BAND_LEN-1),
            (M.w-1) & (BAND_LEN-1) );

        #pragma unroll
        for (uint32 j = 1; j <= BAND_LEN; ++j)
        {
            const simd4u8 value = band[j];
            if (j == M_mod.x+1) r.x = get<0>( value );
            if (j == M_mod.y+1) r.y = get<1>( value );
            if (j == M_mod.z+1) r.z = get<2>( value );
            if (j == M_mod.w+1) r.w = get<3>( value );
        }
        return simd4u8( r );
    }
};

template <uint32 BAND_LEN>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void save_Mth(const uint4 M, const simd4u8* band, simd4u8& best_score)
{
    const simd4u8 m = select_dispatch<BAND_LEN,simd4u8>::enact( M, band );
    best_score = nvbio::max( best_score, m );
}

template <uint32 BAND_LEN, typename sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void save_Mth(const uint4 M, const simd4u8* band, const uint32 i, sink_type& sink, const simd4u8 mask)
{
    const simd4u8 m = select_dispatch<BAND_LEN,simd4u8>::enact( M, band );
    sink.report( and_op( m, mask ) );
}

template <uint32 BAND_LEN, typename score_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void save_Mth(const uint32 M, const score_type* band, score_type& best_score)
{
    const score_type m = select_dispatch<BAND_LEN,score_type>::enact( M, band );
    best_score = nvbio::max( best_score, m );
}

template <uint32 BAND_LEN, typename score_type, typename sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void save_Mth(const uint32 M, const score_type* band, const uint32 i, sink_type& sink)
{
    const score_type m = select_dispatch<BAND_LEN,score_type>::enact( M, band );
    sink.report( m, make_uint2( i+1, M ) );
}
template <uint32 BAND_LEN, typename score_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void save_boundary(const uint32 block, const uint32 M, const score_type* band, score_type& best_score)
{
    if (block + BAND_LEN >= M)
        save_Mth<BAND_LEN>( M, band, best_score );
}
template <uint32 BAND_LEN, typename score_type, typename sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void save_boundary(const uint32 block, const uint32 M, const score_type* band, const uint32 i, sink_type& sink)
{
    if (block + BAND_LEN >= M)
        save_Mth<BAND_LEN>( M, band, i, sink );
}

template <uint32 BAND_LEN, typename sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void save_boundary(const uint32 block, const uint4 M, const simd4u8* band, const uint32 i, sink_type& sink, const simd4u8 active_mask)
{
    const simd4u8 mask = and_op( simd4u8(
        block < M.x && block + BAND_LEN >= M.x ? 0xFFu : 0u ,
        block < M.y && block + BAND_LEN >= M.y ? 0xFFu : 0u ,
        block < M.z && block + BAND_LEN >= M.z ? 0xFFu : 0u ,
        block < M.w && block + BAND_LEN >= M.w ? 0xFFu : 0u ),
        active_mask );

    if (any( mask ))
        save_Mth<BAND_LEN>( M, band, i, sink, mask );
}

} // namespace aln
} // namespace nvbio
