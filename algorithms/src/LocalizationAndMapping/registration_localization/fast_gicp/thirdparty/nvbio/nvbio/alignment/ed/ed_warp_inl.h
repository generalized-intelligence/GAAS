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

#include <nvbio/alignment/sw/sw_warp_inl.h>


namespace nvbio {
namespace aln {
namespace priv {


// private dispatcher for the warp-parallel version of classic smith-waterman
template <
    uint32          BLOCKDIM,
    AlignmentType   TYPE,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        column_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
int32 alignment_score(
    const EditDistanceAligner<TYPE>     aligner,
    const pattern_string                pattern,
    const qual_string                   quals,
    const text_string                   text,
    const  int32                        min_score,
          uint2*                        sink,
          column_type                   column)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return sw_alignment_score<BLOCKDIM,TYPE>(
        EditDistanceSWScheme(),
        pattern,
        quals,
        text,
        min_score,
        sink,
        column );
#else
    return Field_traits<int32>::min();
#endif
}

} // namespace priv
} // namespace aln
} // namespace nvbio
