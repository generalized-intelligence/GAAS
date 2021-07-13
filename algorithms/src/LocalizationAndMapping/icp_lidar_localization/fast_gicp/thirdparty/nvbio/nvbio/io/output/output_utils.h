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

#include <nvbio/io/alignments.h>
#include <nvbio/basic/dna.h>

namespace nvbio {
namespace io {

// compute the CIGAR alignment position given the alignment base and the sink offset
inline uint32 compute_cigar_pos(const uint32 sink, const uint32 alignment)
{
    return alignment + (sink & 0xFFFFu);
}

// compute the reference mapped length from a CIGAR
template <typename vector_type>
uint32 reference_cigar_length(
    const vector_type cigar,
    const uint32      cigar_len)
{
    uint32 r = 0;
    for (uint32 i = 0; i < cigar_len; ++i)
    {
        const uint32 l  = cigar[ cigar_len - i - 1u ].m_len;
        const uint32 op = cigar[ cigar_len - i - 1u ].m_type;
        if (op == Cigar::SUBSTITUTION || op == Cigar::DELETION) r += l;
    }
    return r;
}

// count the symbols of a given type inside a CIGAR
template <typename vector_type>
uint32 count_symbols(
    const Cigar::Operation  type,
    const vector_type       cigar,
    const uint32            cigar_len)
{
    uint32 r = 0;
    for (uint32 i = 0; i < cigar_len; ++i)
    {
        const uint32 l  = cigar[ cigar_len - i - 1u ].m_len;
        const uint32 op = cigar[ cigar_len - i - 1u ].m_type;
        if (op == type) r += l;
    }
    return r;
}

// build the MD string from the internal representation
template <typename vector_type>
void analyze_md_string(const vector_type mds, uint32& n_mm, uint32& n_gapo, uint32& n_gape)
{
    const uint32 mds_len = uint32(mds[0]) | (uint32(mds[1]) << 8);

    n_mm   = 0;
    n_gapo = 0;
    n_gape = 0;

    for (uint32 i = 2; i < mds_len; )
    {
        const uint8 op = mds[i++];
        if (op == MDS_MATCH)
        {
            uint8 l = mds[i++];

            // prolong the MDS match if it spans multiple tokens
            while (i < mds_len && mds[i] == MDS_MATCH)
                l += mds[i++];
        }
        else if (op == MDS_MISMATCH)
        {
            n_mm++;

            ++i;
        }
        else if (op == MDS_INSERTION)
        {
            const uint8 l = mds[i++];

            n_gapo++;
            n_gape += l-1;

            i += l;
        }
        else if (op == MDS_DELETION)
        {
            const uint8 l = mds[i++];

            n_gapo++;
            n_gape += l-1;

            i += l;
        }
    }
}

} // namespace io
} // namespace nvbio
