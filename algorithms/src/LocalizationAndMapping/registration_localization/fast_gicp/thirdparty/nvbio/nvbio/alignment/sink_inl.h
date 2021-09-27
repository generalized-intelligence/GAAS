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
#include <nvbio/basic/simd.h>

namespace nvbio {
namespace aln {

// A sink for valid alignments, mantaining only a single best alignment
//
template <typename ScoreType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
BestSink<ScoreType>::BestSink() : score( Field_traits<ScoreType>::min() ), sink( make_uint2( uint32(-1), uint32(-1) ) ) {}

// invalidate
//
template <typename ScoreType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void BestSink<ScoreType>::invalidate()
{
    score = Field_traits<ScoreType>::min();
    sink  = make_uint2( uint32(-1), uint32(-1) );
}

// store a valid alignment
//
// \param score    alignment's score
// \param sink     alignment's end
//
template <typename ScoreType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void BestSink<ScoreType>::report(const ScoreType _score, const uint2 _sink)
{
    // NOTE: we must use <= here because otherwise we won't pick the bottom-right most one
    // in case there's multiple optimal scores
    if (score <= _score)
    {
        score = _score;
        sink  = _sink;
    }
}

// A sink for valid alignments, mantaining the best two alignments
//
template <typename ScoreType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
Best2Sink<ScoreType>::Best2Sink(const uint32 distinct_dist) :
    score1( Field_traits<ScoreType>::min() ),
    score2( Field_traits<ScoreType>::min() ),
    sink1( make_uint2( uint32(-1), uint32(-1) ) ),
    sink2( make_uint2( uint32(-1), uint32(-1) ) ),
    m_distinct_dist( distinct_dist ) {}

// invalidate
//
template <typename ScoreType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void Best2Sink<ScoreType>::invalidate()
{
    score1 = Field_traits<ScoreType>::min();
    score2 = Field_traits<ScoreType>::min();
    sink1  = make_uint2( uint32(-1), uint32(-1) );
    sink2  = make_uint2( uint32(-1), uint32(-1) );
}

// store a valid alignment
//
// \param score    alignment's score
// \param sink     alignment's end
//
template <typename ScoreType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void Best2Sink<ScoreType>::report(const ScoreType score, const uint2 sink)
{
    // NOTE: we must use <= here because otherwise we won't pick the bottom-right most one
    // in case there's multiple optimal scores
    if (score1 <= score)
    {
        score1 = score;
        sink1  = sink;
    }
    else if (score2 <= score && (sink.x + m_distinct_dist < sink1.x || sink.x > sink1.x + m_distinct_dist))
    {
        score2 = score;
        sink2  = sink;
    }
}

// A sink for valid alignments, mantaining the best two alignments
//
template <typename ScoreType, uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
BestColumnSink<ScoreType,N>::BestColumnSink(const uint32 column_width, const ScoreType min_score) :
    m_column_width( column_width ),
    m_min_score( min_score )
{
    invalidate();
}

// invalidate
//
template <typename ScoreType, uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void BestColumnSink<ScoreType,N>::invalidate()
{
    for (uint32 i = 0; i <= N; ++i)
    {
        scores[i] = Field_traits<ScoreType>::min();
        sinks[i]  = make_uint2( uint32(-1), uint32(-1) );
    }
}

// store a valid alignment
//
// \param score    alignment's score
// \param sink     alignment's end
//
template <typename ScoreType, uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void BestColumnSink<ScoreType,N>::report(const ScoreType score, const uint2 sink)
{
    if (score < m_min_score)
        return;

    const uint32 col = nvbio::min( sink.x / m_column_width, N-1u );

    if (scores[col] <= score)
    {
        scores[col] = score;
        sinks[col]  = sink;
    }
}

// return the index of the best and second-best alignments
//
template <typename ScoreType, uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void BestColumnSink<ScoreType,N>::best2(uint32& i1, uint32& i2, const uint32 min_dist) const
{
    ScoreType s1 = Field_traits<ScoreType>::min();
    ScoreType s2 = Field_traits<ScoreType>::min();
    i1 = N;
    i2 = N;

    // look for the best hit
    for (uint32 i = 0; i < N; ++i)
    {
        if (s1 < scores[i])
        {
            s1 = scores[i];
            i1 = i;
        }
    }

    // check whether we found a valid score
    if (s1 == Field_traits<ScoreType>::min())
        return;

    // look for the second hit, at a minimum distance from the best
    for (uint32 i = 0; i < N; ++i)
    {
        if ((s2 < scores[i]) &&
            ((sinks[i].x + min_dist <= sinks[i1].x) ||
             (sinks[i].x            >= sinks[i1].x + min_dist)))
        {
            s2 = scores[i];
            i2 = i;
        }
    }
}

} // namespace aln
} // namespace nvbio
