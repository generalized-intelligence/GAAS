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
#include <nvbio/alignment/utils.h>

namespace nvbio {
namespace aln {

namespace priv {

template <
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        column_type>
struct alignment_score_dispatch {};

template <
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        column_type>
struct alignment_checkpointed_dispatch {};

template <
    uint32          BAND_LEN,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string>
struct banded_alignment_score_dispatch {};

template <
    uint32          BAND_LEN,
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string>
struct banded_alignment_checkpointed_dispatch {};

//
// Helper class for banded alignment
//
template <uint32 BAND_SIZE> struct Reference_cache
{
    static const uint32 BAND_WORDS = (BAND_SIZE-1+7) >> 3;
    typedef PackedStream<uint32*,uint8,2,false> type;
};
template <> struct Reference_cache<15u>
{
    static const uint32 BAND_WORDS = 14u;
    typedef uint32* type;
};
template <> struct Reference_cache<7u>
{
    static const uint32 BAND_WORDS = 6u;
    typedef uint32* type;
};
template <> struct Reference_cache<5u>
{
    static const uint32 BAND_WORDS = 4u;
    typedef uint32* type;
};
template <> struct Reference_cache<3u>
{
    static const uint32 BAND_WORDS = 3u;
    typedef uint32* type;
};

} // namespace priv

} // namespace alignment
} // namespace nvbio
