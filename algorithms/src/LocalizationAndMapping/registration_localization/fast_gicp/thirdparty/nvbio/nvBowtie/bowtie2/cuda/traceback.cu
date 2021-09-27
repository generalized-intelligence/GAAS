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

#include <nvBowtie/bowtie2/cuda/traceback.h>
#include <nvBowtie/bowtie2/cuda/traceback_impl.h>
#include <nvBowtie/bowtie2/cuda/pipeline_states.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

//
// execute a batch of banded-alignment traceback calculations
//
void banded_traceback_best(
    const uint32                                                aln_idx,
    const uint32                                                count,
    const uint32*                                               idx,
          io::Alignment*                                        best_data,
    const uint32                                                best_stride,
    const uint32                                                band_len,
    const TracebackPipelineState<EditDistanceScoringScheme>&    pipeline,
    const ParamsPOD&                                            params)
{
    if (aln_idx)
        banded_traceback_best_t<1>( count, idx, best_data, best_stride, band_len, pipeline, params );
    else
        banded_traceback_best_t<0>( count, idx, best_data, best_stride, band_len, pipeline, params );
}

//
// execute a batch of banded-alignment traceback calculations
//
void banded_traceback_best(
    const uint32                                                    aln_idx,
    const uint32                                                    count,
    const uint32*                                                   idx,
          io::Alignment*                                            best_data,
    const uint32                                                    best_stride,
    const uint32                                                    band_len,
    const TracebackPipelineState<SmithWatermanScoringScheme<> >&    pipeline,
    const ParamsPOD&                                                params)
{
    if (aln_idx)
        banded_traceback_best_t<1>( count, idx, best_data, best_stride, band_len, pipeline, params );
    else
        banded_traceback_best_t<0>( count, idx, best_data, best_stride, band_len, pipeline, params );
}

//
// execute a batch of opposite alignment traceback calculations
//
void opposite_traceback_best(
    const uint32                                                aln_idx,
    const uint32                                                count,
    const uint32*                                               idx,
          io::Alignment*                                        best_data,
    const uint32                                                best_stride,
    const TracebackPipelineState<EditDistanceScoringScheme>&    pipeline,
    const ParamsPOD&                                            params)
{
    if (aln_idx)
        opposite_traceback_best_t<1>( count, idx, best_data, best_stride, pipeline, params );
    else
        opposite_traceback_best_t<0>( count, idx, best_data, best_stride, pipeline, params );
}

//
// execute a batch of opposite alignment traceback calculations
//
void opposite_traceback_best(
    const uint32                                                    aln_idx,
    const uint32                                                    count,
    const uint32*                                                   idx,
          io::Alignment*                                            best_data,
    const uint32                                                    best_stride,
    const TracebackPipelineState<SmithWatermanScoringScheme<> >&    pipeline,
    const ParamsPOD&                                                params)
{
    if (aln_idx)
        opposite_traceback_best_t<1>( count, idx, best_data, best_stride, pipeline, params );
    else
        opposite_traceback_best_t<0>( count, idx, best_data, best_stride, pipeline, params );
}


//
// execute a batch of banded-alignment traceback calculations
//
void banded_traceback_all(
    const uint32                                                count,
    const uint32*                                               idx,
    const uint32                                                buffer_offset,
    const uint32                                                buffer_size,
          io::Alignment*                                        alignments,
    const uint32                                                band_len,
    const AllMappingPipelineState<EditDistanceScoringScheme>&   pipeline,
    const ParamsPOD&                                            params)
{
    banded_traceback_all_t( count, idx, buffer_offset, buffer_size, alignments, band_len, pipeline, params );
}

//
// execute a batch of banded-alignment traceback calculations
//
void banded_traceback_all(
    const uint32                                                    count,
    const uint32*                                                   idx,
    const uint32                                                    buffer_offset,
    const uint32                                                    buffer_size,
          io::Alignment*                                            alignments,
    const uint32                                                    band_len,
    const AllMappingPipelineState<SmithWatermanScoringScheme<> >&   pipeline,
    const ParamsPOD&                                                params)
{
    banded_traceback_all_t( count, idx, buffer_offset, buffer_size, alignments, band_len, pipeline, params );
}

//
// finish a batch of alignment calculations
//
void finish_alignment_best(
    const uint32                                                aln_idx,
    const uint32                                                count,
    const uint32*                                               idx,
          io::Alignment*                                        best_data,
    const uint32                                                best_stride,
    const uint32                                                band_len,
    const TracebackPipelineState<EditDistanceScoringScheme>&    pipeline,
    const SmithWatermanScoringScheme<>                          scoring_scheme,
    const ParamsPOD&                                            params)
{
    if (aln_idx)
        finish_alignment_best_t<1>( count, idx, best_data, best_stride, band_len, pipeline, scoring_scheme, params );
    else
        finish_alignment_best_t<0>( count, idx, best_data, best_stride, band_len, pipeline, scoring_scheme, params );
}

//
// finish a batch of alignment calculations
//
void finish_alignment_best(
    const uint32                                                    aln_idx,
    const uint32                                                    count,
    const uint32*                                                   idx,
          io::Alignment*                                            best_data,
    const uint32                                                    best_stride,
    const uint32                                                    band_len,
    const TracebackPipelineState<SmithWatermanScoringScheme<> >&    pipeline,
    const SmithWatermanScoringScheme<>                              scoring_scheme,
    const ParamsPOD&                                                params)
{
    if (aln_idx)
        finish_alignment_best_t<1>( count, idx, best_data, best_stride, band_len, pipeline, scoring_scheme, params );
    else
        finish_alignment_best_t<0>( count, idx, best_data, best_stride, band_len, pipeline, scoring_scheme, params );
}

//
// finish a batch of opposite alignment calculations
//
void finish_opposite_alignment_best(
    const uint32                                                aln_idx,
    const uint32                                                count,
    const uint32*                                               idx,
          io::Alignment*                                        best_data,
    const uint32                                                best_stride,
    const uint32                                                band_len,
    const TracebackPipelineState<EditDistanceScoringScheme>&    pipeline,
    const SmithWatermanScoringScheme<>                          scoring_scheme,
    const ParamsPOD&                                            params)
{
    if (aln_idx)
        finish_opposite_alignment_best_t<1>( count, idx, best_data, best_stride, band_len, pipeline, scoring_scheme, params );
    else
        finish_opposite_alignment_best_t<0>( count, idx, best_data, best_stride, band_len, pipeline, scoring_scheme, params );
}

//
// finish a batch of opposite alignment calculations
//
void finish_opposite_alignment_best(
    const uint32                                                    aln_idx,
    const uint32                                                    count,
    const uint32*                                                   idx,
          io::Alignment*                                            best_data,
    const uint32                                                    best_stride,
    const uint32                                                    band_len,
    const TracebackPipelineState<SmithWatermanScoringScheme<> >&    pipeline,
    const SmithWatermanScoringScheme<>                              scoring_scheme,
    const ParamsPOD&                                                params)
{
    if (aln_idx)
        finish_opposite_alignment_best_t<1>( count, idx, best_data, best_stride, band_len, pipeline, scoring_scheme, params );
    else
        finish_opposite_alignment_best_t<0>( count, idx, best_data, best_stride, band_len, pipeline, scoring_scheme, params );
}

//
// finish a batch of alignment calculations, all-mapping
//
void finish_alignment_all(
    const uint32                                                count,
    const uint32*                                               idx,
    const uint32                                                buffer_offset,
    const uint32                                                buffer_size,
          io::Alignment*                                        alignments,
    const uint32                                                band_len,
    const AllMappingPipelineState<EditDistanceScoringScheme>&   pipeline,
    const SmithWatermanScoringScheme<>                          scoring_scheme,
    const ParamsPOD&                                            params)
{
    finish_alignment_all_t( count, idx, buffer_offset, buffer_size, alignments, band_len, pipeline, scoring_scheme, params );
}

//
// finish a batch of alignment calculations, all-mapping
//
void finish_alignment_all(
    const uint32                                                    count,
    const uint32*                                                   idx,
    const uint32                                                    buffer_offset,
    const uint32                                                    buffer_size,
          io::Alignment*                                            alignments,
    const uint32                                                    band_len,
    const AllMappingPipelineState<SmithWatermanScoringScheme<> >&   pipeline,
    const SmithWatermanScoringScheme<>                              scoring_scheme,
    const ParamsPOD&                                                params)
{
    finish_alignment_all_t( count, idx, buffer_offset, buffer_size, alignments, band_len, pipeline, scoring_scheme, params );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio

#include <nvBowtie/bowtie2/cuda/traceback_inl.h>
