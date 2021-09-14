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

// alignment.h
//

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <nvbio/alignment/alignment.h>
#include <nvbio/alignment/batched.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/io/sequence/sequence.h>

using namespace nvbio;

// a functor to extract the read infixes from the hit diagonals
//
struct read_infixes
{
    // constructor
    NVBIO_HOST_DEVICE
    read_infixes(const io::ConstSequenceDataView reads) :
        m_reads( reads ) {}

    // functor operator
    NVBIO_HOST_DEVICE
    string_infix_coord_type operator() (const uint2 diagonal) const
    {
        const io::SequenceDataAccess<DNA_N> reads( m_reads );

        const uint32 read_id = diagonal.y;

        // fetch the read range
        return reads.get_range( read_id );
    }

    const io::ConstSequenceDataView m_reads;
};

// a functor to extract the genome infixes from the hit diagonals
//
template <uint32 BAND_LEN>
struct genome_infixes
{
    // constructor
    NVBIO_HOST_DEVICE
    genome_infixes(const uint32 genome_len, const io::ConstSequenceDataView reads) :
        m_genome_len( genome_len ),
        m_reads( reads ) {}

    // functor operator
    NVBIO_HOST_DEVICE
    string_infix_coord_type operator() (const uint2 diagonal) const
    {
        const io::SequenceDataAccess<DNA_N> reads( m_reads );

        const uint32 read_id  = diagonal.y;
        const uint32 text_pos = diagonal.x;

        // fetch the read range
        const uint2  read_range = reads.get_range( read_id );
        const uint32 read_len   = read_range.y - read_range.x;

        // compute the segment of text to align to
        const uint32 genome_begin = text_pos > BAND_LEN/2 ? text_pos - BAND_LEN/2 : 0u;
        const uint32 genome_end   = nvbio::min( genome_begin + read_len + BAND_LEN, m_genome_len );

        return make_uint2( genome_begin, genome_end );
    }

    const uint32                    m_genome_len;
    const io::ConstSequenceDataView m_reads;
};

// a functor to extract the score from a sink
//
struct sink_score
{
    typedef aln::BestSink<int16> argument_type;
    typedef int16                result_type;

    // functor operator
    NVBIO_HOST_DEVICE
    int16 operator() (const aln::BestSink<int16>& sink) const { return sink.score; }
};
