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

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvBowtie/bowtie2/cuda/string_utils.h>
#include <nvBowtie/bowtie2/cuda/scoring.h>
#include <nvbio/alignment/alignment.h>
#include <nvbio/alignment/batched.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/io/alignments.h>
#include <nvbio/io/utils.h>
#include <nvbio/basic/dna.h> // for dna_to_char

namespace nvbio {
namespace bowtie2 {
namespace cuda {

///@addtogroup nvBowtie
///@{

///@addtogroup Traceback
///@{

template <typename vector_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 read_cigar_length(
    const vector_type cigar,
    const uint32      cigar_len)
{
    uint32 r = 0;
    for (uint32 i = 0; i < cigar_len; ++i)
    {
        const uint32 l  = cigar[ cigar_len - i - 1u ].m_len;
        const uint32 op = cigar[ cigar_len - i - 1u ].m_type;
        if (op != io::Cigar::DELETION) r += l;
    }
    return r;
}

enum MateType
{
    AnchorMate   = 0,
    OppositeMate = 1,
};

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
const char* mate_string(const MateType mate) { return mate == OppositeMate ? "opposite" : "anchor"; }

///
/// execute a batch of banded-alignment traceback calculations
///
template <uint32 ALN_IDX, typename pipeline_type>
void banded_traceback_best_t(
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const ParamsPOD             params);

///
/// execute a batch of opposite alignment traceback calculations
///
template <uint32 ALN_IDX, typename pipeline_type>
void opposite_traceback_best_t(
    const uint32                count,
    const uint32*               idx,
          io::Alignment*       best_data,
    const uint32                best_stride,
    const pipeline_type&        pipeline,
    const ParamsPOD             params);

///
/// execute a batch of banded-alignment traceback calculations
///
template <typename pipeline_type>
void banded_traceback_all_t(
    const uint32                count,
    const uint32*               idx,
    const uint32                buffer_offset,
    const uint32                buffer_size,
          io::Alignment*        alignments,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const ParamsPOD             params);

///
/// finish a batch of alignment calculations
///
template <uint32 ALN_IDX, typename scoring_scheme_type, typename pipeline_type>
void finish_alignment_best_t(
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const scoring_scheme_type   scoring_scheme,
    const ParamsPOD             params);

///
/// finish a batch of opposite alignment calculations
///
template <uint32 ALN_IDX, typename scoring_scheme_type, typename pipeline_type>
void finish_opposite_alignment_best_t(
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const scoring_scheme_type   scoring_scheme,
    const ParamsPOD             params);

///
/// finish a batch of alignment calculations, all-mapping
///
template <typename scoring_scheme_type, typename pipeline_type>
void finish_alignment_all_t(
    const uint32                count,
    const uint32*               idx,
    const uint32                buffer_offset,
    const uint32                buffer_size,
          io::Alignment*        alignments,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const scoring_scheme_type   scoring_scheme,
    const ParamsPOD             params);

///@}  // group Traceback
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio

#include <nvBowtie/bowtie2/cuda/traceback_inl.h>
